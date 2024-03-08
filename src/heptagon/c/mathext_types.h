#ifndef MATHEXT_H
#define MATHEXT_H

typedef struct Mathext__float_out {
  float y;
} Mathext__float_out;

void Mathext__float_step(int x, Mathext__float_out* _out);

typedef struct Mathext__round_out {
  int y;
} Mathext__round_out;

void Mathext__round_step(float x, Mathext__round_out* _out);

typedef struct Mathext__ceil_out {
  float y;
} Mathext__ceil_out;

void Mathext__ceil_step(float x, Mathext__ceil_out* _out);

typedef struct Mathext__floor_out {
  float y;
} Mathext__floor_out;

void Mathext__floor_step(float x, Mathext__floor_out* _out);

typedef struct Mathext__sin_out {
  float y;
} Mathext__sin_out;

void Mathext__sin_step(float x, Mathext__sin_out* _out);

typedef struct Mathext__cos_out {
  float y;
} Mathext__cos_out;

void Mathext__cos_step(float x, Mathext__cos_out* _out);

typedef struct Mathext__tan_out {
  float y;
} Mathext__tan_out;

void Mathext__tan_step(float x, Mathext__tan_out* _out);

typedef struct Mathext__asin_out {
  float y;
} Mathext__asin_out;

void Mathext__asin_step(float x, Mathext__asin_out* _out);

typedef struct Mathext__acos_out {
  float y;
} Mathext__acos_out;

void Mathext__acos_step(float x, Mathext__acos_out* _out);

typedef struct Mathext__atan_out {
  float y;
} Mathext__atan_out;

void Mathext__atan_step(float x, Mathext__atan_out* _out);

typedef struct Mathext__atan2_out {
  float z;
} Mathext__atan2_out;

void Mathext__atan2_step(float x, float y, Mathext__atan2_out* _out);

typedef struct Mathext__sqrt_out {
  float y;
} Mathext__sqrt_out;

void Mathext__sqrt_step(float x, Mathext__sqrt_out* _out);

typedef struct Mathext__fabs_out {
  float y;
} Mathext__fabs_out;

void Mathext__fabs_step(float x, Mathext__fabs_out* _out);

typedef struct Mathext__min_float_out {
  float z;
} Mathext__min_float_out;

void Mathext__min_float_step(float x, float y, Mathext__min_float_out* _out);

typedef struct Mathext__max_float_out {
  float z;
} Mathext__max_float_out;

void Mathext__max_float_step(float x, float y, Mathext__max_float_out* _out);

typedef struct Mathext__power_out {
  float r;
} Mathext__power_out;

void Mathext__power_step(float x, float y, Mathext__power_out *o);

typedef struct Mathext__fmod_out {
  float z;
} Mathext__fmod_out;

void Mathext__fmod_step(float x, float y, Mathext__fmod_out* _out);

typedef struct Mathext__invSqrt_out {
  float y;
} Mathext__invSqrt_out;

void Mathext__invSqrt_step(float x, Mathext__invSqrt_out* _out);

typedef struct Mathext__xTaskGetTickCount_out {
  int tick;
} Mathext__xTaskGetTickCount_out;

void Mathext__xTaskGetTickCount_step(Mathext__xTaskGetTickCount_out* _out);

#include <stdint.h>
uint32_t xTaskGetTickCount( void );

// kalman estimator
typedef struct Mathext__arm_sqrt_out {
  float y;
} Mathext__arm_sqrt_out;

void Mathext__arm_sqrt_step(float x, Mathext__arm_sqrt_out* _out);

typedef struct Mathext__arm_cos_f32_out {
  float y;
} Mathext__arm_cos_f32_out;

void Mathext__arm_cos_f32_step(float x, Mathext__arm_cos_f32_out* _out);

typedef struct Mathext__arm_sin_f32_out {
  float y;
} Mathext__arm_sin_f32_out;

void Mathext__arm_sin_f32_step(float x, Mathext__arm_sin_f32_out* _out);

typedef struct Mathext__quadrocopter_state {
    float kc_state_x;
    float kc_state_y;
    float kc_state_z;
    float kc_state_px;
    float kc_state_py;
    float kc_state_pz;
    float kc_state_d0;
    float kc_state_d1;
    float kc_state_d2;
} Mathext__quadrocopter_state;

typedef struct Mathext__covariance_matrix {
    Mathext__quadrocopter_state kc_state_X;
    Mathext__quadrocopter_state kc_state_Y;
    Mathext__quadrocopter_state kc_state_Z;
    Mathext__quadrocopter_state kc_state_PX;
    Mathext__quadrocopter_state kc_state_PY;
    Mathext__quadrocopter_state kc_state_PZ;
    Mathext__quadrocopter_state kc_state_D0;
    Mathext__quadrocopter_state kc_state_D1;
    Mathext__quadrocopter_state kc_state_D2;
} Mathext__covariance_matrix;


typedef struct Mathext__quaternion  {
    float qx;
    float qy;
    float qz;
    float qw
} Mathext__quaternion;

typedef struct Mathext__kalman_coredata_t {
    Mathext__quadrocopter_state s;
    Mathext__quaternion q;
    float r[3][3];
    Mathext__covariance_matrix p;
    Mathext__quaternion initial_quaternion;
} Mathext__kalman_coredata_t;

typedef struct Mathext__covariance_update_out {
  Mathext__covariance_matrix p;
} Mathext__covariance_update_out;

void quadrocopter_to_array(Mathext__quadrocopter_state* q, float p_array[9]);
void array_to_quadrocopter(float p_array[9], Mathext__quadrocopter_state* q);
void covariance_matrix_to_matrix(Mathext__covariance_matrix* p, float p_array[9][9]);
void matrix_to_covariance_matrix(float p_array[9][9], Mathext__covariance_matrix* p);

void Mathext__covariance_update_step(Mathext__covariance_matrix am, Mathext__covariance_matrix p, Mathext__covariance_update_out* _out);

typedef struct Mathext__enforce_covariance_matrix_symmetry_out {
  Mathext__covariance_matrix p;
} Mathext__enforce_covariance_matrix_symmetry_out;

void Mathext__enforce_covariance_matrix_symmetry_step(Mathext__covariance_matrix p, Mathext__enforce_covariance_matrix_symmetry_out* _out);

typedef struct Mathext__supervisor_is_flying_out {
  int ok;
} Mathext__supervisor_is_flying_out;

void Mathext__supervisor_is_flying_step(Mathext__supervisor_is_flying_out* _out);

typedef struct Mathext__relay_state_out {
  int ok;
} Mathext__relay_state_out;

void Mathext__relay_state_step(Mathext__quadrocopter_state s, Mathext__relay_state_out* _out);

typedef struct Mathext__relay_covariance_matrix_out {
  int ok;
} Mathext__relay_covariance_matrix_out;

void Mathext__relay_covariance_matrix_step(Mathext__covariance_matrix p, Mathext__relay_covariance_matrix_out* _out);


typedef struct Mathext__kalman_core_scalar_update_out {
  Mathext__kalman_coredata_t core_data_updated;
} Mathext__kalman_core_scalar_update_out;
void Mathext__kalman_core_scalar_update_step(Mathext__kalman_coredata_t this, Mathext__quadrocopter_state h, float error, float stdMeasNoise, Mathext__kalman_core_scalar_update_out* _out);


#endif // MATHEXT_H
