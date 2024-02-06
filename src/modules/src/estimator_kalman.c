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
static bool updateQueuedMeasurements(const uint32_t tick);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, 2*KALMAN_TASK_STACKSIZE);

#define LOG_LENGTH 1000
#define LOG_RATE 20
#define DELAY LOG_RATE/10
static TickType_t waitTime;
static TickType_t* waitTimes[LOG_LENGTH];
static TickType_t startTime;
static TickType_t* startTimes[LOG_LENGTH];
static TickType_t endTime;
static TickType_t* endTimes[LOG_LENGTH];
static uint16_t* iterations[LOG_LENGTH];


static Libel__kalman_coredata_t kalman_coredata;
static Libel__vec3 acc_accumulator;
static Libel__vec3 gyro_accumulator;
static Libel__vec3 acc_latest;
static Libel__predict_state_forward_out predict_state_forward_out;
static Libel__predict_state_forward_mem predict_state_forward_mem;

// --------------------------------------------------

// Libel Functions -----------

void libel_from_axis3f_kalman(Axis3f *in, Libel__vec3 *out) {
  out->x = in->x;
  out->y = in->y;
  out->z = in->z;
}

void libel_to_axis3f_kalman(Libel__vec3 *in, Axis3f *out) {
  out->x = in->x;
  out->y = in->y;
  out->z = in->z;
}

void libel_from_quaternion_kalman(float in[4], Libel__quaternion *out) {
  out->qx = in[0];
  out->qy = in[1];
  out->qz = in[2];
  out->qw = in[3];
}

void libel_to_quaternion_kalman(Libel__quaternion *in, float out[4]) {
  out[0] = in->qx;
  out[1] = in->qy;
  out[2] = in->qz;
  out[3] = in->qw;
}

void libel_quadrocopter_to_array(Mathext__quadrocopter_state* q, float p_array[9]) {
  p_array[0] = q->kc_state_x;
  p_array[1] = q->kc_state_y;
  p_array[2] = q->kc_state_z;
  p_array[3] = q->kc_state_px;
  p_array[4] = q->kc_state_py;
  p_array[5] = q->kc_state_pz;
  p_array[6] = q->kc_state_d0;
  p_array[7] = q->kc_state_d1;
  p_array[8] = q->kc_state_d2;
}

void libel_covariance_matrix_to_matrix(Mathext__covariance_matrix* p, float p_array[9][9]) {
  libel_quadrocopter_to_array(&p->kc_state_X, p_array[0]);
  libel_quadrocopter_to_array(&p->kc_state_Y, p_array[1]);
  libel_quadrocopter_to_array(&p->kc_state_Z, p_array[2]);
  libel_quadrocopter_to_array(&p->kc_state_PX, p_array[3]);
  libel_quadrocopter_to_array(&p->kc_state_PY, p_array[4]);
  libel_quadrocopter_to_array(&p->kc_state_PZ, p_array[5]);
  libel_quadrocopter_to_array(&p->kc_state_D0, p_array[6]);
  libel_quadrocopter_to_array(&p->kc_state_D1, p_array[7]);
  libel_quadrocopter_to_array(&p->kc_state_D2, p_array[8]);
}

void libel_array_to_quadrocpter(float p_array[9], Mathext__quadrocopter_state* q) {
  q->kc_state_x = p_array[0];
  q->kc_state_y = p_array[1];
  q->kc_state_z = p_array[2];
  q->kc_state_px = p_array[3];
  q->kc_state_py = p_array[4];
  q->kc_state_pz = p_array[5];
  q->kc_state_d0 = p_array[6];
  q->kc_state_d1 = p_array[7];
  q->kc_state_d2 = p_array[8];
}

void libel_matrix_to_covariance_matrix(float p_array[9][9], Mathext__covariance_matrix* p) {
  libel_array_to_quadrocpter(p_array[0], &p->kc_state_X);
  libel_array_to_quadrocpter(p_array[1], &p->kc_state_Y);
  libel_array_to_quadrocpter(p_array[2], &p->kc_state_Z);
  libel_array_to_quadrocpter(p_array[3], &p->kc_state_PX);
  libel_array_to_quadrocpter(p_array[4], &p->kc_state_PY);
  libel_array_to_quadrocpter(p_array[5], &p->kc_state_PZ);
  libel_array_to_quadrocpter(p_array[6], &p->kc_state_D0);
  libel_array_to_quadrocpter(p_array[7], &p->kc_state_D1);
  libel_array_to_quadrocpter(p_array[8], &p->kc_state_D2);
}

void libel_from_coreData(kalmanCoreData_t *in, Libel__kalman_coredata_t *out){
  libel_array_to_quadrocpter(in->S, &out->s);
  libel_from_quaternion_kalman(in->q, &out->q);
  memcpy( out->r, in->R,sizeof(float)*3*3);
  libel_matrix_to_covariance_matrix(in->P, &out->p);
  libel_from_quaternion_kalman(in->initialQuaternion, &out->initial_quaternion);
}

void libel_to_coreData(Libel__kalman_coredata_t *in, kalmanCoreData_t *out){
  libel_quadrocopter_to_array(&in->s, out->S);
  libel_to_quaternion_kalman(&in->q, out->q);
  memcpy(out->R, in->r,  sizeof(float)*3*3);
  libel_covariance_matrix_to_matrix(&in->p, out->P);
  libel_to_quaternion_kalman(&in->initial_quaternion, out->initialQuaternion);
}

void libel_to_attitude_kalman(Libel__attitude *in, attitude_t *out) {
    out->roll = in->roll;
    out->pitch = in->pitch;
    out->yaw = in->yaw;
}

void libel_to_vec3_kalman(Libel__vec3 *in, struct vec3_s *out) {
    out->x = in->x;
    out->y = in->y;
    out->z = in->z;
}

void libel_to_quaternion2_kalman(Libel__quaternion *in, quaternion_t *out) {
    out->x = in->qx;
    out->y = in->qy;
    out->z = in->qz;
    out->w = in->qw;
}

void libel_to_state_kalman(Libel__state_t *in, state_t *out){
  libel_to_attitude_kalman(&in->st_attitude, &out->attitude);
  libel_to_quaternion2_kalman(&in->st_attitude_quat, &out->attitudeQuaternion);
  libel_to_vec3_kalman(&in->st_position, &out->position);
  libel_to_vec3_kalman(&in->st_velocity, &out->velocity);
  libel_to_vec3_kalman(&in->st_acc, &out->acc);
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
  int count = 0;
  while (true) {
    //DEBUG_PRINT("Kalman now\n") ;
    waitTime = usecTimestamp();
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
        kalmanCoreAddProcessNoise(&coreData, &coreParams, dt);
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
      kalmanCoreFinalize(&coreData, osTick);
      STATS_CNT_RATE_EVENT(&finalizeCounter);
      if (! kalmanSupervisorIsStateWithinBounds(&coreData)) {
        resetEstimation = true;

        if (osTick > warningBlockTime) {
          warningBlockTime = osTick + WARNING_HOLD_BACK_TIME;
          DEBUG_PRINT("State out of bounds, resetting\n");
        }
      }
    }

    /**
     * Finally, the internal state is externalized.
     * This is done every round, since the external state includes some sensor data
     */
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accLatest, osTick);
    xSemaphoreGive(dataMutex);

    STATS_CNT_RATE_EVENT(&updateCounter);
    endTime = usecTimestamp();
    
    if(count>=0 && count/LOG_RATE == LOG_LENGTH){
      // reset buffer when finished reading
      count = 0;
    }
    if(count >= 0 && count<LOG_LENGTH){
      // fill buffer
      startTimes[count] = startTime;
      endTimes[count] = endTime;
      waitTimes[count] = waitTime;
      iterations[count] = count;
    }
    if(count>=0 && count%LOG_RATE == LOG_RATE-1){
      // get next timestamp every lograte loops
      for(int i =1; i < LOG_LENGTH; i++){
        startTimes[i-1] = startTimes[i];
        endTimes[i-1] = endTimes[i];
        waitTimes[i-1] = waitTimes[i];
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
  libel_from_axis3f_kalman(&accAccumulator, &acc_accumulator);
  libel_from_axis3f_kalman(&gyroAccumulator, &gyro_accumulator);
  libel_from_coreData(&coreData, &kalman_coredata);
  Libel__predict_state_forward_step(
    kalman_coredata,
    (float)gyroAccumulatorCount, (float)accAccumulatorCount,
    gyro_accumulator,
    acc_accumulator,  
    &predict_state_forward_out, &predict_state_forward_mem
  );


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
  kalmanCorePredict(&coreData, &accAverage, &gyroAverage, dt, quadIsFlying);

  return true;
}


static bool updateQueuedMeasurements(const uint32_t tick) {
  bool doneUpdate = false;
  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */

  // Pull the latest sensors values of interest; discard the rest
  measurement_t m;
  while (estimatorDequeue(&m)) {
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
        kalmanCoreUpdateWithTof(&coreData, &m.data.tof);
        doneUpdate = true;
        break;
      case MeasurementTypeAbsoluteHeight:
        kalmanCoreUpdateWithAbsoluteHeight(&coreData, &m.data.height);
        doneUpdate = true;
        break;
      case MeasurementTypeFlow:
        kalmanCoreUpdateWithFlow(&coreData, &m.data.flow, &gyroLatest);
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
        gyroAccumulator.x += m.data.gyroscope.gyro.x;
        gyroAccumulator.y += m.data.gyroscope.gyro.y;
        gyroAccumulator.z += m.data.gyroscope.gyro.z;
        gyroLatest = m.data.gyroscope.gyro;
        gyroAccumulatorCount++;
        break;
      case MeasurementTypeAcceleration:
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
LOG_ADD(LOG_UINT32, kalmanwait, &waitTimes[0])
LOG_ADD(LOG_UINT32, kalmanstart, &startTimes[0])
LOG_ADD(LOG_UINT32, kalmanend, &endTimes[0])
LOG_ADD(LOG_UINT16, kalmanloop, &iterations[0])
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
