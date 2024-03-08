/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 * 2021.03.15, Wolfgang Hoenig: Refactored queue handling
 */

#include "kalman_core.h"
#include "kalman_supervisor.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"
#include "supervisor.h"

#include "statsCnt.h"
#include "rateSupervisor.h"

// Measurement models
#include "mm_distance.h"
#include "mm_absolute_height.h"
#include "mm_position.h"
#include "mm_pose.h"
#include "mm_tdoa.h"
#include "mm_flow.h"
#include "mm_tof.h"
#include "mm_yaw_error.h"
#include "mm_sweep_angles.h"

#include "mm_tdoa_robust.h"
#include "mm_distance_robust.h"

#define DEBUG_MODULE "ESTKALMAN"
#include "debug.h"

#include "libel.h"
#include "libel_types.h"

#include "arm_math.h"

#include "tita_debug_functions.h"

// #define KALMAN_USE_BARO_UPDATE


// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Use the robust implementations of TWR and TDoA, off by default but can be turned on through a parameter.
// The robust implementations use around 10% more CPU VS the standard flavours
static bool robustTwr = false;
static bool robustTdoa = false;

/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 *
 * For more information, refer to the paper
 */

NO_DMA_CCM_SAFE_ZERO_INIT static kalmanCoreData_t coreData;


/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;

static Axis3f accAccumulator;
static Axis3f gyroAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static Axis3f accLatest;
static Axis3f gyroLatest;
static bool quadIsFlying = false;

static OutlierFilterLhState_t sweepOutlierFilterState;

// Indicates that the internal state is corrupt and should be reset
bool resetEstimation = false;

static kalmanCoreParams_t coreParams;

// Data used to enable the task and stabilizer loop to run with minimal locking
static state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilzer when needed.

// Statistics
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

static rateSupervisor_t rateSupervisorContext;

#define WARNING_HOLD_BACK_TIME M2T(2000)
static uint32_t warningBlockTime = 0;

#ifdef KALMAN_USE_BARO_UPDATE
static const bool useBaroUpdate = true;
#else
static const bool useBaroUpdate = false;
#endif

static void kalmanTask(void* parameters);
static bool predictStateForward(uint32_t osTick, float dt);
bool updateQueuedMeasurements(const uint32_t tick);

#define KALMAN_TASK_STACKSIZE_OLD 450
#define KALMAN_TASK_STACKSIZE_NEW 1400
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, KALMAN_TASK_STACKSIZE_NEW);

#define LOG_LENGTH 1000
#define LOG_RATE 20
#define DELAY LOG_RATE/10
#define MAX_COUNT 256
static TickType_t startTime =0;
static TickType_t* startTimes[LOG_LENGTH];
static TickType_t endTime =0;
static TickType_t* endTimes[LOG_LENGTH];
static uint8_t* iterations[LOG_LENGTH];

static TickType_t tmpTime=0;
static TickType_t c_predStFwdTime =0;
static TickType_t c_procNoiseTime=0;
static TickType_t c_finalizeTime=0;
static TickType_t c_externalizeTime=0;
static TickType_t c_stateInBoundsTime=0;
static TickType_t libel_predStFwdTime =0;
static TickType_t libel_procNoiseTime=0;
static TickType_t libel_finalizeTime=0;
static TickType_t libel_externalizeTime=0;
static TickType_t libel_stateInBoundsTime=0;



static Libel__flow libel_flow;
static Libel__tof libel_tof;
static Libel__vec3 libel_acc;
static Libel__vec3 libel_gyro;


NO_DMA_CCM_SAFE_ZERO_INIT static float am_temp[KC_STATE_DIM][KC_STATE_DIM];
static __attribute__((aligned(4))) arm_matrix_instance_f32 Am = { KC_STATE_DIM, KC_STATE_DIM, (float *)am_temp};
 

// -------------------------------------------------------
// Debug Functions ---------------------------------------

#define DEBUG_FIRST_N 5
#define DEBUG_BLOCK_SIZE 50
#define DEBUG_EVERY_N 500


#define DEBUG_PREDICT_STATE_FORWARD false;
#define DEBUG_ADD_PROCESS_NOISE false;
#define DEBUG_FINALIZE false;
#define DEBUG_EXTERNALIZE false;
#define DEBUG_STATE_WITHIN_BOUNDS false;
#define DEBUG_UPDATE_MEASUREMENT true;



static int debug_predict_state_forward = false;
static int debug_add_process_noise = false;
static int debug_finalize = false;
static int debug_externalize = false;
static int debug_state_within_bounds = false;
static int debug_update_measurement = false;




static int debug_count = 0;
int do_print_debug(){
  debug_count++;
  if(debug_count%DEBUG_EVERY_N<DEBUG_BLOCK_SIZE){
    debug_externalize = DEBUG_EXTERNALIZE;
    debug_add_process_noise = DEBUG_ADD_PROCESS_NOISE;
    debug_predict_state_forward = DEBUG_PREDICT_STATE_FORWARD;
    debug_finalize = DEBUG_FINALIZE;
    debug_state_within_bounds = DEBUG_STATE_WITHIN_BOUNDS;
    debug_update_measurement = DEBUG_UPDATE_MEASUREMENT;

    char predict_names[128] = "";
    if(debug_add_process_noise){
      strcat(predict_names, "PrNoise ");
    }
    if(debug_predict_state_forward){
      strcat(predict_names, "PredStateFwd ");
    }
    if(debug_finalize){
      strcat(predict_names, "Finalize ");
    }
    if(debug_externalize){
      strcat(predict_names, "Externalize ");
    }
    if(debug_state_within_bounds){
      strcat(predict_names, "StateInBounds ");
    }
    if(debug_update_measurement){
      strcat(predict_names, "UpdateMeasurement");
    }
    //DEBUG_PRINT("------%i DEBUG %i ( %s) ----\n",KALMAN_TASK_STACKSIZE_NEW, debug_count,predict_names);  
    return true;
  }
  debug_externalize = false;
  debug_add_process_noise = false;
  debug_predict_state_forward = false;
  debug_finalize = false;
  debug_state_within_bounds = false;
  debug_update_measurement = false;
}

// Debugging libel
static kalmanCoreData_t libel_coredata;
static Mathext__kalman_coredata_t kalman_coredata;
static float S_libel[9];
void relayLibelState(float S[9]){
  for(int i=0; i<9; i++){
    S_libel[i] = S[i];
  }
}

static float P_libel[9][9];
static arm_matrix_instance_f32 Pm_libel;


void relayLibelCovarianceMatrix(float P[9][9]){
  Pm_libel.numCols =9;
  Pm_libel.numRows = 9;
  Pm_libel.pData = (float*) P_libel;
  for(int i = 0; i<9; i++){
    for(int j = 0; j <9; j++){
      P_libel[i][j] = P[i][j];
    }
  }
  return;
}


// Called one time during system startup
void estimatorKalmanTaskInit() {
  kalmanCoreDefaultParams(&coreParams);

  vSemaphoreCreateBinary(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(kalmanTask, kalmanTask, KALMAN_TASK_NAME, NULL, KALMAN_TASK_PRI);

  isInit = true;
}

bool estimatorKalmanTaskTest() {
  return isInit;
}

static void kalmanTask(void* parameters) {
  systemWaitStart();

  uint32_t lastPrediction = xTaskGetTickCount();
  uint32_t nextPrediction = xTaskGetTickCount();
  uint32_t lastPNUpdate = xTaskGetTickCount();

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), ONE_SECOND, PREDICT_RATE - 1, PREDICT_RATE + 1, 1);
  int count = 512;
  while (true) {
    do_print_debug();
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    startTime = usecTimestamp();
    // If the client triggers an estimator reset via parameter update
    if (resetEstimation) {
      estimatorKalmanInit();
      resetEstimation = false;
    }

    // Tracks whether an update to the state has been made, and the state therefore requires finalization
    bool doneUpdate = false;

    uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

  #ifdef KALMAN_DECOUPLE_XY
    kalmanCoreDecoupleXY(&coreData);
  #endif

    // Run the system dynamics to predict the state forward.
    if (osTick >= nextPrediction) { // update at the PREDICT_RATE
      float dt = T2S(osTick - lastPrediction);
      if (predictStateForward(osTick, dt)) {
        lastPrediction = osTick;
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&predictionCounter);
      }

      nextPrediction = osTick + S2T(1.0f / PREDICT_RATE);

      if (!rateSupervisorValidate(&rateSupervisorContext, T2M(osTick))) {
        DEBUG_PRINT("WARNING: Kalman prediction rate low (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
      }
    }

    /**
     * Add process noise every loop, rather than every prediction
     */
    {
      float dt = T2S(osTick - lastPNUpdate);
      if (dt > 0.0f) {
        dt = 0.001f;
        libel_from_coreData(&coreData,&kalman_coredata);

        tmpTime = usecTimestamp();
        kalmanCoreAddProcessNoise(&coreData, &coreParams, dt);
        c_procNoiseTime = usecTimestamp() - tmpTime;
        
        if(debug_add_process_noise){
          Libel__kalman_core_add_process_noise_out noise_out;
          tmpTime = usecTimestamp();
          Libel__kalman_core_add_process_noise_step(
            kalman_coredata,
            &noise_out
          );
          libel_procNoiseTime = usecTimestamp() - tmpTime;
          libel_to_coreData(&noise_out.core_data_final, &libel_coredata);
          equal_coredata("AddProcessNoise", &coreData, &libel_coredata);
        }
        
        lastPNUpdate = osTick;
      }
    }

    {
      if(updateQueuedMeasurements(osTick)) {
        doneUpdate = true;
      }
    }

    /**
     * If an update has been made, the state is finalized:
     * - the attitude error is moved into the body attitude quaternion,
     * - the body attitude is converted into a rotation matrix for the next prediction, and
     * - correctness of the covariance matrix is ensured
     */

    if (doneUpdate)
    {
      if(debug_finalize || debug_state_within_bounds)
        libel_from_coreData(&coreData,&kalman_coredata);
      
      tmpTime = usecTimestamp();
      kalmanCoreFinalize(&coreData, osTick);
      c_finalizeTime = usecTimestamp() - tmpTime;
      STATS_CNT_RATE_EVENT(&finalizeCounter);
      
      tmpTime = usecTimestamp();
      int isOk = kalmanSupervisorIsStateWithinBounds(&coreData);
      c_stateInBoundsTime = usecTimestamp() - tmpTime;
      if(debug_state_within_bounds){
        Libel__kalman_supervisor_is_state_within_bounds_out is_state_within_bounds_out;
        tmpTime = usecTimestamp();
        Libel__kalman_supervisor_is_state_within_bounds_step(
          kalman_coredata,
          &is_state_within_bounds_out
        );
        libel_stateInBoundsTime = usecTimestamp() - tmpTime;
        int is_ok = is_state_within_bounds_out.ok;
        if(is_ok != isOk){
          DEBUG_PRINT("ERROR in State within bounds! Expected: %i Got: %i", isOk, is_ok);
        }
      }

      if (! isOk) {
        resetEstimation = true;

        if (osTick > warningBlockTime) {
          warningBlockTime = osTick + WARNING_HOLD_BACK_TIME;
          DEBUG_PRINT("State out of bounds, resetting\n");
        }
      }

      if(debug_finalize){
        Libel__kalman_core_finalize_out finalize_out;
        Libel__kalman_core_finalize_mem finalize_mem;
        tmpTime = usecTimestamp();
        Libel__kalman_core_finalize_step(
          kalman_coredata,
          &finalize_out, &finalize_mem
        );
        libel_finalizeTime = usecTimestamp() - tmpTime;
        libel_to_coreData(&finalize_out.this_final, &libel_coredata);
        equal_coredata("Finalize",&coreData, &libel_coredata);
        }
    }

    /**
     * Finally, the internal state is externalized.
     * This is done every round, since the external state includes some sensor data
     */
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    if(debug_externalize){
      libel_from_coreData(&coreData,&kalman_coredata);  
    }
    tmpTime = usecTimestamp();
    kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accLatest, osTick);
    c_externalizeTime = usecTimestamp()- tmpTime;
    xSemaphoreGive(dataMutex);

    if(debug_externalize){

      Libel__vec3 acc_latest;
      libel_from_axis3f_kalman(&accLatest, &acc_latest);
      Libel__kalman_core_externalize_state_out externalize_out;
      tmpTime = usecTimestamp();
      Libel__kalman_core_externalize_state_step(
        kalman_coredata,
        acc_latest,
        &externalize_out
      );
      libel_externalizeTime = usecTimestamp()- tmpTime;
      state_t task_estimator_state;
      libel_to_state_kalman(&externalize_out.st, &task_estimator_state);
      equal_state(&taskEstimatorState, &task_estimator_state);
    }

    STATS_CNT_RATE_EVENT(&updateCounter);
    endTime = usecTimestamp();
    
    if(count>=0 && count/LOG_RATE == LOG_LENGTH){
      // reset buffer when finished reading
      count = 0;
    }
    if(count >= 0 && count<LOG_LENGTH){
      // fill buffer
      *startTimes[count] = startTime;
      *endTimes[count] = endTime;
      *iterations[count] = count%MAX_COUNT;
    }
    if(count>=0 && count%LOG_RATE == LOG_RATE-1){
      // get next timestamp every lograte loops
      for(int i =1; i < LOG_LENGTH; i++){
        startTimes[i-1] = startTimes[i];
        endTimes[i-1] = endTimes[i];
        iterations[i-1] = iterations[i];
      }
    }
    count++;

  }
}

void estimatorKalman(state_t *state, const uint32_t tick)
{
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // Copy the latest state, calculated by the task
  memcpy(state, &taskEstimatorState, sizeof(state_t));
  xSemaphoreGive(dataMutex);

  xSemaphoreGive(runTaskSemaphore);
}

static bool predictStateForward(uint32_t osTick, float dt) {
  dt = 0.01;
  
  tmpTime = usecTimestamp();
  if(false){
    Libel__vec3 acc_accumulator;
    Libel__vec3 gyro_accumulator;
    libel_from_coreData(&coreData, &kalman_coredata);
    libel_from_axis3f_kalman(&accAccumulator, &acc_accumulator);
    libel_from_axis3f_kalman(&gyroAccumulator, &gyro_accumulator);
    Libel__predict_state_forward_out predict_state_forward_out;
    Libel__predict_state_forward_mem predict_state_forward_mem;
    
    Libel__predict_state_forward_step(
      kalman_coredata, 
      (float) gyroAccumulatorCount,
      (float) accAccumulatorCount,
      gyro_accumulator,
      acc_accumulator,
      &predict_state_forward_out, 
      &predict_state_forward_mem
    );
    libel_predStFwdTime = usecTimestamp() - tmpTime;
    libel_to_coreData(&predict_state_forward_out.core_data_updated, &libel_coredata);
  }
  tmpTime = usecTimestamp();
  if (gyroAccumulatorCount == 0
      || accAccumulatorCount == 0)
  {
    return false;
  }

  // gyro is in deg/sec but the estimator requires rad/sec
  Axis3f gyroAverage;
  gyroAverage.x = gyroAccumulator.x * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage.y = gyroAccumulator.y * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage.z = gyroAccumulator.z * DEG_TO_RAD / gyroAccumulatorCount;

  // accelerometer is in Gs but the estimator requires ms^-2
  Axis3f accAverage;
  accAverage.x = accAccumulator.x * GRAVITY_MAGNITUDE / accAccumulatorCount;
  accAverage.y = accAccumulator.y * GRAVITY_MAGNITUDE / accAccumulatorCount;
  accAverage.z = accAccumulator.z * GRAVITY_MAGNITUDE / accAccumulatorCount;

  // reset for next call
  accAccumulator = (Axis3f){.axis={0}};
  accAccumulatorCount = 0;
  gyroAccumulator = (Axis3f){.axis={0}};
  gyroAccumulatorCount = 0;

  quadIsFlying = supervisorIsFlying();
  
  if(debug_predict_state_forward){
    Libel__vec3 acc;
    Libel__vec3 gyro;
    libel_from_coreData(&coreData, &kalman_coredata);
    libel_from_axis3f_kalman(&accAverage, &acc);
    libel_from_axis3f_kalman(&gyroAverage, &gyro);
    Libel__kalman_core_predict_out kalman_core_predict_out;
    Libel__kalman_core_predict_mem kalman_core_predict_mem;
    
    Libel__kalman_core_predict_step(
      kalman_coredata, 
      acc, gyro, 
      dt, (int) quadIsFlying,
      &kalman_core_predict_out, 
      &kalman_core_predict_mem
    );
    libel_to_coreData(&kalman_core_predict_out.this_updated, &libel_coredata);
  }
  kalmanCorePredict(&coreData, &accAverage, &gyroAverage, dt, quadIsFlying, Pm_libel, S_libel, false);
  c_predStFwdTime = usecTimestamp() - tmpTime;
  if(debug_predict_state_forward){
    equal_coredata("KalmanCorePredict",&coreData, &libel_coredata);
  }
  return true;
}


bool updateQueuedMeasurements(const uint32_t tick) {
  bool doneUpdate = false;
  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */

  // Pull the latest sensors values of interest; discard the rest
  
  measurement_t m;
  while (estimatorDequeue(&m)) {
    if(debug_update_measurement)
      libel_from_coreData(&coreData, &kalman_coredata);
    bool c_flow = false;
    bool c_tof = false;
    bool c_acc = false;
    bool c_gyro = false;
    Axis3f gyroAccumulator_old = gyroAccumulator;
    Axis3f gyroLatest_old = gyroLatest;
    int gyroAccumulatorCount_old = gyroAccumulatorCount;
    Axis3f accAccumulator_old = accAccumulator;
    Axis3f accLatest_old = accLatest;
    int accAccumulatorCount_old = accAccumulatorCount;
    switch (m.type) {
      case MeasurementTypeTDOA:
        if(robustTdoa){
          // robust KF update with TDOA measurements
          kalmanCoreRobustUpdateWithTDOA(&coreData, &m.data.tdoa);
        }else{
          // standard KF update
          kalmanCoreUpdateWithTDOA(&coreData, &m.data.tdoa);
        }
        doneUpdate = true;
        break;
      case MeasurementTypePosition:
        kalmanCoreUpdateWithPosition(&coreData, &m.data.position);
        doneUpdate = true;
        break;
      case MeasurementTypePose:
        kalmanCoreUpdateWithPose(&coreData, &m.data.pose);
        doneUpdate = true;
        break;
      case MeasurementTypeDistance:
        if(robustTwr){
            // robust KF update with UWB TWR measurements
            kalmanCoreRobustUpdateWithDistance(&coreData, &m.data.distance);
        }else{
            // standard KF update
            kalmanCoreUpdateWithDistance(&coreData, &m.data.distance);
        }
        doneUpdate = true;
        break;
      case MeasurementTypeTOF:
        c_tof = true;
        libel_tof = *((Libel__tof*)  &m.data.tof);
        kalmanCoreUpdateWithTof(&coreData, &m.data.tof);
        doneUpdate = true;
        break;
      case MeasurementTypeAbsoluteHeight:
        kalmanCoreUpdateWithAbsoluteHeight(&coreData, &m.data.height);
        doneUpdate = true;
        break;
      case MeasurementTypeFlow:
        c_flow = true;
        libel_flow = *((Libel__flow*)  &(m.data.flow));
        kalmanCoreUpdateWithFlow(&coreData, &m.data.flow, &gyroLatest, S_libel);
        doneUpdate = true;
        break;
      case MeasurementTypeYawError:
        kalmanCoreUpdateWithYawError(&coreData, &m.data.yawError);
        doneUpdate = true;
        break;
      case MeasurementTypeSweepAngle:
        kalmanCoreUpdateWithSweepAngles(&coreData, &m.data.sweepAngle, tick, &sweepOutlierFilterState);
        doneUpdate = true;
        break;
      case MeasurementTypeGyroscope:
        c_gyro = true;
        libel_gyro = *((Libel__vec3*)  &m.data.gyroscope.gyro);
        gyroAccumulator.x += m.data.gyroscope.gyro.x;
        gyroAccumulator.y += m.data.gyroscope.gyro.y;
        gyroAccumulator.z += m.data.gyroscope.gyro.z;
        gyroLatest = m.data.gyroscope.gyro;
        gyroAccumulatorCount++;
        break;
      case MeasurementTypeAcceleration:
        c_acc = true;
        libel_acc = *((Libel__vec3*)  &m.data.acceleration.acc);
        accAccumulator.x += m.data.acceleration.acc.x;
        accAccumulator.y += m.data.acceleration.acc.y;
        accAccumulator.z += m.data.acceleration.acc.z;
        accLatest = m.data.acceleration.acc;
        accAccumulatorCount++;
        break;
      case MeasurementTypeBarometer:
        if (useBaroUpdate) {
          kalmanCoreUpdateWithBaro(&coreData, &coreParams, m.data.barometer.baro.asl, quadIsFlying);
          doneUpdate = true;
        }
        break;
      default:
        break;
    }
    if(debug_update_measurement){
      //DEBUG_PRINT("c_flow %d, c_tof %d, c_acc %d, c_gyro %d\n", (int) c_flow, (int) c_tof, (int) c_acc, (int) c_gyro);
      Libel__update_measurement_out update_out;
      Libel__update_measurement_step(kalman_coredata, c_flow, c_tof, c_acc, c_gyro, libel_flow, libel_tof, libel_acc, libel_gyro, *((Libel__vec3*) &accAccumulator_old),*((Libel__vec3*) &gyroAccumulator_old), (float) accAccumulatorCount_old, (float) gyroAccumulatorCount_old, *((Libel__vec3*) &accLatest_old), *((Libel__vec3*) &gyroLatest_old), &update_out);
      libel_to_coreData(&update_out.core_data_updated, &libel_coredata);
      equal_float_array("accAccumulator", &update_out.acc_accumulator_updated, &accAccumulator, 3);
      equal_float_array("gyroAccumulator", &update_out.gyro_accumulator_updated, &gyroAccumulator, 3);
      equal_float_array("accLatest", &update_out.acc_latest_updated, &accLatest, 3);
      equal_float_array("gyroLatest", &update_out.gyro_latest_updated, &gyroLatest, 3);
      equal_floats("accAccumulatorCount", update_out.acc_accumulator_count_updated, (float) accAccumulatorCount);
      equal_floats("gyroAccumulatorCount", update_out.gyro_accumulator_count_updated, (float) gyroAccumulatorCount);
      if(c_acc)
        equal_coredata("UpdateAcc", &coreData, &libel_coredata);
      else if(c_flow)
        equal_coredata("UpdateFlow", &coreData, &libel_coredata);
      else if(c_gyro)
        equal_coredata("UpdateGyro", &coreData, &libel_coredata);
      else if(c_tof)
        equal_coredata("UpdateTOF", &coreData, &libel_coredata);
      else
        equal_coredata("UpdateNone", &coreData, &libel_coredata);
    }
  }
  
  return doneUpdate;
}

// Called when this estimator is activated
void estimatorKalmanInit(void)
{
  accAccumulator = (Axis3f){.axis = {0}};
  gyroAccumulator = (Axis3f){.axis = {0}};

  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;
  outlierFilterReset(&sweepOutlierFilterState, 0);

  kalmanCoreInit(&coreData, &coreParams);
  kalmanCoreInit(&libel_coredata, &coreParams);
  DEBUG_PRINT("-------INIT DEBUG--------\n");
  equal_coredata("Init", &coreData, &libel_coredata);
}

bool estimatorKalmanTest(void)
{
  return isInit;
}

void estimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = coreData.S[KC_STATE_X];
  pos->y = coreData.S[KC_STATE_Y];
  pos->z = coreData.S[KC_STATE_Z];
}

void estimatorKalmanGetEstimatedRot(float * rotationMatrix) {
  memcpy(rotationMatrix, coreData.R, 9*sizeof(float));
}

LOG_GROUP_START(timer)
LOG_ADD(LOG_UINT32, kalmanstart, &startTimes[0])
LOG_ADD(LOG_UINT32, kalmanend, &endTimes[0])
LOG_ADD(LOG_UINT16, processnoise_libel, &libel_procNoiseTime)

//LOG_ADD(LOG_UINT16, predictstateforward_libel, &libel_predStFwdTime)

LOG_ADD(LOG_UINT16, finalize_libel, &libel_finalizeTime)
LOG_ADD(LOG_UINT16, externalize_libel, &libel_externalizeTime)
LOG_ADD(LOG_UINT16, stateinbounds_libel, &libel_stateInBoundsTime)

LOG_ADD(LOG_UINT16, processnoise_c, &c_procNoiseTime)
//LOG_ADD(LOG_UINT16, predictstateforward_c, &c_predStFwdTime)
LOG_ADD(LOG_UINT16, finalize_c, &c_finalizeTime)
LOG_ADD(LOG_UINT16, externalize_c, &c_externalizeTime)
LOG_ADD(LOG_UINT16, stateinbounds_c, &c_stateInBoundsTime) 
LOG_GROUP_STOP(timer)

/**
 * Variables and results from the Extended Kalman Filter
 */
LOG_GROUP_START(kalman)
/**
 * @brief Nonzero if the drone is in flight
 *
 *  Note: This is the same as sys.flying. Perhaps remove this one?
 */
  LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
  /**
 * @brief State position in the global frame x
 *
 *   Note: This is similar to stateEstimate.x.
 */
  LOG_ADD(LOG_FLOAT, stateX, &coreData.S[KC_STATE_X])
 /**
 * @brief State position in the global frame y
 *
 *  Note: This is similar to stateEstimate.y
 */
  LOG_ADD(LOG_FLOAT, stateY, &coreData.S[KC_STATE_Y])
 /**
 * @brief State position in the global frame z
 *
 *  Note: This is similar to stateEstimate.z
 */
  LOG_ADD(LOG_FLOAT, stateZ, &coreData.S[KC_STATE_Z])
  /**
 * @brief State position in the global frame PX
 *
 *  Note: This is similar to stateEstimate.x
 */
  LOG_ADD(LOG_FLOAT, statePX, &coreData.S[KC_STATE_PX])
  /**
  * @brief State velocity in its body frame y
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePY, &coreData.S[KC_STATE_PY])
  /**
  * @brief State velocity in its body frame z
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePZ, &coreData.S[KC_STATE_PZ])
  /**
  * @brief State attitude error roll
  */
  LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
  /**
  * @brief State attitude error pitch
  */
  LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
  /**
  * @brief State attitude error yaw
  */
  LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
  /**
  * @brief Covariance matrix position x
  */
  LOG_ADD(LOG_FLOAT, varX, &coreData.P[KC_STATE_X][KC_STATE_X])
  /**
  * @brief Covariance matrix position y
  */
  LOG_ADD(LOG_FLOAT, varY, &coreData.P[KC_STATE_Y][KC_STATE_Y])
  /**
  * @brief Covariance matrix position z
  */
  LOG_ADD(LOG_FLOAT, varZ, &coreData.P[KC_STATE_Z][KC_STATE_Z])
  /**
  * @brief Covariance matrix velocity x
  */
  LOG_ADD(LOG_FLOAT, varPX, &coreData.P[KC_STATE_PX][KC_STATE_PX])
  /**
  * @brief Covariance matrix velocity y
  */
  LOG_ADD(LOG_FLOAT, varPY, &coreData.P[KC_STATE_PY][KC_STATE_PY])
  /**
  * @brief Covariance matrix velocity z
  */
  LOG_ADD(LOG_FLOAT, varPZ, &coreData.P[KC_STATE_PZ][KC_STATE_PZ])
  /**
  * @brief Covariance matrix attitude error roll
  */
  LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
  /**
  * @brief Covariance matrix attitude error pitch
  */
  LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
  /**
  * @brief Covariance matrix attitude error yaw
  */
  LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
  /**
  * @brief Estimated Attitude quarternion w
  */
  LOG_ADD(LOG_FLOAT, q0, &coreData.q[0])
  /**
  * @brief Estimated Attitude quarternion x
  */
  LOG_ADD(LOG_FLOAT, q1, &coreData.q[1])
  /**
  * @brief Estimated Attitude quarternion y
  */
  LOG_ADD(LOG_FLOAT, q2, &coreData.q[2])
  /**
  * @brief Estimated Attitude quarternion z
  */
  LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])
  /**
  * @brief Statistics rate of update step
  */
  STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  /**
  * @brief Statistics rate of prediction step
  */
  STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
  /**
  * @brief Statistics rate full estimation step
  */
  STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
LOG_GROUP_STOP(kalman)

LOG_GROUP_START(outlierf)
  LOG_ADD(LOG_INT32, lhWin, &sweepOutlierFilterState.openingWindow)
LOG_GROUP_STOP(outlierf)

/**
 * Tuning parameters for the Extended Kalman Filter (EKF)
 *     estimator
 */
PARAM_GROUP_START(kalman)
/**
 * @brief Reset the kalman estimator
 */
  PARAM_ADD_CORE(PARAM_UINT8, resetEstimation, &resetEstimation)
  PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
/**
 * @brief Nonzero to use robust TDOA method (default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, robustTdoa, &robustTdoa)
/**
 * @brief Nonzero to use robust TWR method (default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, robustTwr, &robustTwr)
/**
 * @brief Process noise for x and y acceleration
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAcc_xy, &coreParams.procNoiseAcc_xy)
/**
 * @brief Process noise for z acceleration
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAcc_z, &coreParams.procNoiseAcc_z)
  /**
 * @brief Process noise for velocity
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNVel, &coreParams.procNoiseVel)
  /**
 * @brief Process noise for position
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNPos, &coreParams.procNoisePos)
  /**
 * @brief Process noise for attitude
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAtt, &coreParams.procNoiseAtt)
  /**
 * @brief Measurement noise for barometer
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNBaro, &coreParams.measNoiseBaro)
  /**
 * @brief Measurement noise for roll/pitch gyros
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNGyro_rollpitch, &coreParams.measNoiseGyro_rollpitch)
  /**
 * @brief Measurement noise for yaw gyro
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNGyro_yaw, &coreParams.measNoiseGyro_yaw)
  /**
 * @brief Initial X after reset [m]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialX, &coreParams.initialX)
  /**
 * @brief Initial Y after reset [m]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialY, &coreParams.initialY)
  /**
 * @brief Initial Z after reset [m]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialZ, &coreParams.initialZ)
  /**
 * @brief Initial yaw after reset [rad]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialYaw, &coreParams.initialYaw)
PARAM_GROUP_STOP(kalman)
