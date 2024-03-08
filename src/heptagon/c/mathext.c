#include <math.h>
#include "mathext_types.h"
//#include "FreeRTOS.h"
#include <stdint.h>
// #include "task.h"

void Mathext__float_step(int x, Mathext__float_out* _out) {
  _out->y = (float)x;
}

void Mathext__round_step(float x, Mathext__round_out* _out) {
  _out->y = (int)lroundf(x);
}

void Mathext__ceil_step(float x, Mathext__ceil_out* _out) {
  _out->y = ceilf(x);
}

void Mathext__floor_step(float x, Mathext__floor_out* _out) {
  _out->y = floorf(x);
}

void Mathext__sin_step(float x, Mathext__sin_out* _out) {
  _out->y = sinf(x);
}

void Mathext__cos_step(float x, Mathext__cos_out* _out) {
  _out->y = cosf(x);
}

void Mathext__tan_step(float x, Mathext__tan_out* _out) {
  _out->y = tanf(x);
}

void Mathext__asin_step(float x, Mathext__asin_out* _out) {
  _out->y = asinf(x);
}

void Mathext__acos_step(float x, Mathext__acos_out* _out) {
  _out->y = acosf(x);
}

void Mathext__atan_step(float x, Mathext__atan_out* _out) {
  _out->y = atanf(x);
}

void Mathext__atan2_step(float x, float y, Mathext__atan2_out* _out) {
  _out->z = atan2f(x, y);
}

void Mathext__sqrt_step(float x, Mathext__sqrt_out* _out) {
  _out->y = sqrt(x);
}

void Mathext__fabs_step(float x, Mathext__fabs_out* _out) {
  _out->y = fabsf(x);
}

void Mathext__min_float_step(float x, float y, Mathext__min_float_out* _out) {
  _out->z = (x < y)? x : y;
}

void Mathext__max_float_step(float x, float y, Mathext__max_float_out* _out) {
  _out->z = (x > y)? x : y;
}

void Mathext__power_step(float x, float y, Mathext__power_out *out)
{
  out->r = powf(x, y);
}

void Mathext__fmod_step(float x, float y, Mathext__fmod_out* _out) {
  _out->z = fmod(x, y);
}

void Mathext__invSqrt_step(float x, Mathext__invSqrt_out* _out) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  _out->y = y;
}

void Mathext__xTaskGetTickCount_step(Mathext__xTaskGetTickCount_out* _out) {
  _out->tick = xTaskGetTickCount();
}

// Kalman filter
#include "static_mem.h"
#include "arm_math.h"
#include "kalman_core.h"

void Mathext__arm_sqrt_step(float x, Mathext__arm_sqrt_out* _out) {
  _out->y = arm_sqrt(x);
}

void Mathext__arm_cos_f32_step(float x, Mathext__arm_cos_f32_out* _out) {
  _out->y = arm_cos_f32(x);
}

void Mathext__arm_sin_f32_step(float x, Mathext__arm_sin_f32_out* _out) {
  _out->y = arm_sin_f32(x);
}

void quadrocopter_to_array(Mathext__quadrocopter_state* q, float p_array[9]) {
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

void covariance_matrix_to_matrix(Mathext__covariance_matrix* p, float p_array[9][9]) {
  quadrocopter_to_array(&p->kc_state_X, p_array[0]);
  quadrocopter_to_array(&p->kc_state_Y, p_array[1]);
  quadrocopter_to_array(&p->kc_state_Z, p_array[2]);
  quadrocopter_to_array(&p->kc_state_PX, p_array[3]);
  quadrocopter_to_array(&p->kc_state_PY, p_array[4]);
  quadrocopter_to_array(&p->kc_state_PZ, p_array[5]);
  quadrocopter_to_array(&p->kc_state_D0, p_array[6]);
  quadrocopter_to_array(&p->kc_state_D1, p_array[7]);
  quadrocopter_to_array(&p->kc_state_D2, p_array[8]);
}

void array_to_quadrocopter(float p_array[9], Mathext__quadrocopter_state* q) {
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

void matrix_to_covariance_matrix(float p_array[9][9], Mathext__covariance_matrix* p) {
  array_to_quadrocopter(p_array[0], &p->kc_state_X);
  array_to_quadrocopter(p_array[1], &p->kc_state_Y);
  array_to_quadrocopter(p_array[2], &p->kc_state_Z);
  array_to_quadrocopter(p_array[3], &p->kc_state_PX);
  array_to_quadrocopter(p_array[4], &p->kc_state_PY);
  array_to_quadrocopter(p_array[5], &p->kc_state_PZ);
  array_to_quadrocopter(p_array[6], &p->kc_state_D0);
  array_to_quadrocopter(p_array[7], &p->kc_state_D1);
  array_to_quadrocopter(p_array[8], &p->kc_state_D2);
}

void covariance_update(Mathext__covariance_matrix* am, Mathext__covariance_matrix* p) {

  __attribute__((aligned(4))) static float p_array[KC_STATE_DIM][KC_STATE_DIM];
  __attribute__((aligned(4))) static float am_array[KC_STATE_DIM][KC_STATE_DIM];
  // convert Mathext__covariance_matrix to matrix
  covariance_matrix_to_matrix(p, p_array);
  covariance_matrix_to_matrix(am, am_array);

  // Initialize arm_matrix_instances for the matrices
  static arm_matrix_instance_f32 Am = {KC_STATE_DIM, KC_STATE_DIM, (float *)am_array};
  static arm_matrix_instance_f32 Pm = {KC_STATE_DIM, KC_STATE_DIM, (float *)p_array};

  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&Am, &Pm, &tmpNN1m); // A P
  mat_mult(&tmpNN1m, &tmpNN2m, &Pm); // A P A'

  // convert matrix to Mathext__covariance_matrix
  matrix_to_covariance_matrix(p_array, p);
}

void Mathext__covariance_update_step(Mathext__covariance_matrix am, Mathext__covariance_matrix p, Mathext__covariance_update_out* _out) {
  // update covariance matrix
  covariance_update(&am, &p);
  _out->p = p;
}

#define MAX_COVAR (100)
#define MIN_COVAR (1e-6f)

void Mathext__enforce_covariance_matrix_symmetry_step(Mathext__covariance_matrix p, Mathext__enforce_covariance_matrix_symmetry_out* _out) {
  __attribute__((aligned(4))) float p_array[KC_STATE_DIM][KC_STATE_DIM];
  covariance_matrix_to_matrix(&p, p_array);

  // enforce symmetry of the covariance matrix, and ensure the values stay bounded
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float p_val = 0.5f*p_array[i][j] + 0.5f*p_array[j][i];
      if (isnan(p_val) || p_val > MAX_COVAR) {
        p_array[i][j] = p_array[j][i] = MAX_COVAR;
      } else if ( i==j && p_val < MIN_COVAR ) {
        p_array[i][j] = p_array[j][i] = MIN_COVAR;
      } else {
        p_array[i][j] = p_array[j][i] = p_val;
      }
    }
  }
  _out->p = p;
  
}

#include "supervisor.h"
void Mathext__supervisor_is_flying_step(Mathext__supervisor_is_flying_out* _out) {
  _out->ok = supervisorIsFlying();
}



#include "debug.h"
#include "estimator_kalman.h"
void Mathext__relay_state_step(Mathext__quadrocopter_state s, Mathext__relay_state_out* _out){
  float S[9];
  quadrocopter_to_array(&s, S);
  relayLibelState(S);
  _out->ok = true;
  }

void Mathext__relay_covariance_matrix_step(Mathext__covariance_matrix p, Mathext__relay_covariance_matrix_out* _out){
  float P[9][9];
  covariance_matrix_to_matrix(&p, P);
  relayLibelCovarianceMatrix(P);
  _out->ok = true;
}



// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

void Mathext__kalman_core_scalar_update_step(Mathext__kalman_coredata_t this, Mathext__quadrocopter_state h, float error, float stdMeasNoise, Mathext__kalman_core_scalar_update_out* _out){
  // convert lustre types to c types
  __attribute__((aligned(4))) static float hm_array[KC_STATE_DIM];
  quadrocopter_to_array(&h, &hm_array);
  static arm_matrix_instance_f32 Hm = {1, KC_STATE_DIM, (float *)hm_array};

  __attribute__((aligned(4))) static float S[KC_STATE_DIM];
  quadrocopter_to_array(&this.s, &S);

  __attribute__((aligned(4))) static float P[KC_STATE_DIM][KC_STATE_DIM];
  covariance_matrix_to_matrix(&this.p, P);
  static arm_matrix_instance_f32 Pm = {KC_STATE_DIM, KC_STATE_DIM, (float *)P};
  


  // The Kalman gain as a column vector
  NO_DMA_CCM_SAFE_ZERO_INIT static float K[KC_STATE_DIM];
  static arm_matrix_instance_f32 Km = {KC_STATE_DIM, 1, (float *)K};

  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN3m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN3d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float HTd[KC_STATE_DIM * 1];
  static arm_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float PHTd[KC_STATE_DIM * 1];
  static arm_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};

  ASSERT(Hm.numRows == 1);
  ASSERT(Hm.numCols == KC_STATE_DIM);

  // ====== INNOVATION COVARIANCE ======

  mat_trans(&Hm, &HTm);
  mat_mult(&Pm, &HTm, &PHTm); // PH'
  float R = stdMeasNoise*stdMeasNoise;
  float HPHR = R; // HPH' + R
  for (int i=0; i<KC_STATE_DIM; i++) { // Add the element of HPH' to the above
    HPHR += Hm.pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
  }
  ASSERT(!isnan(HPHR));

  // ====== MEASUREMENT UPDATE ======
  // Calculate the Kalman gain and perform the state update
  for (int i=0; i<KC_STATE_DIM; i++) {
    K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    S[i] = S[i] + K[i] * error; // state update
  }

  // ====== COVARIANCE UPDATE ======
  mat_mult(&Km, &Hm, &tmpNN1m); // KH
  for (int i=0; i<KC_STATE_DIM; i++) { tmpNN1d[KC_STATE_DIM*i+i] -= 1; } // KH - I
  mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I)*P
  mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I)*P*(KH - I)'

  // add the measurement variance and ensure boundedness and symmetry
  // TODO: Why would it hit these bounds? Needs to be investigated.
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float v = K[i] * R * K[j];
      float p = 0.5f*P[i][j] + 0.5f*P[j][i] + v; // add measurement noise
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }


  // convert back to lustre types
  array_to_quadrocopter(S,&this.s);
  matrix_to_covariance_matrix(P,&this.p);
  _out->core_data_updated = this;
}