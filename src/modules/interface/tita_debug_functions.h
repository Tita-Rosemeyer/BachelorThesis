#include "libel.h"
#include "libel_types.h"

#include "debug.h"
#include "supervisor.h"
#include "kalman_core.h"
#include "arm_math.h"
#include <stdio.h>
#define EPS 0.001f

int equal_floats(char *name, float f1, float f2);
int equal_axis3f(char *varname, Axis3f ref, Axis3f got);
int equal_int16(const char *varname, int16_t i1, int16_t i2) ;
int equal_attitude(const attitude_t *attitude1, const attitude_t *attitude2);
int equal_attitudeQuaternion(const quaternion_t *attitudeQuaternion1, const quaternion_t *attitudeQuaternion2);
int equal_position(const point_t *position1, const point_t *position2);
int equal_velocity(const velocity_t *velocity1, const velocity_t *velocity2);
int equal_acc(const acc_t *acc1, const acc_t *acc2);
void debug_print_state(state_t *struc);
int equal_state(state_t *state1, state_t *state2);
int equal_controls(const control_t *control1, const control_t *control2) ;
int equal_float_array(const char* varname, float* ref, float *got, int n);
int equal_arm_matrix_f32(const char* varname, arm_matrix_instance_f32 ref, arm_matrix_instance_f32 got);
int equal_coredata(const char* debug_name, kalmanCoreData_t *refcore, kalmanCoreData_t* gotcore);

void libel_from_axis3f_kalman(Axis3f *in, Libel__vec3 *out) ;
void libel_to_axis3f_kalman(Libel__vec3 *in, Axis3f *out);
void libel_from_quaternion_kalman(float in[4], Mathext__quaternion *out) ;
void libel_to_quaternion_kalman(Mathext__quaternion *in, float out[4]) ;
void libel_quadrocopter_to_array(Mathext__quadrocopter_state* q, float p_array[9]);
void libel_covariance_matrix_to_matrix(Mathext__covariance_matrix* p, float p_array[9][9]) ;
void libel_array_to_quadrocopter(float p_array[9], Mathext__quadrocopter_state* q) ;
void libel_matrix_to_covariance_matrix(float p_array[9][9], Mathext__covariance_matrix* p);
void libel_from_coreData(kalmanCoreData_t *in, Mathext__kalman_coredata_t *out);
void libel_to_coreData(Mathext__kalman_coredata_t *in, kalmanCoreData_t *out);

void libel_to_attitude_kalman(Libel__attitude *in, attitude_t *out) ;
void libel_to_vec3_kalman(Libel__vec3 *in, struct vec3_s *out);
void libel_to_quaternion2_kalman(Mathext__quaternion *in, quaternion_t *out) ;
void libel_to_state_kalman(Libel__state_t *in, state_t *out);

void libel_to_tof(Libel__tof *in , tofMeasurement_t *out);
void libel_from_tof(tofMeasurement_t *in , Libel__tof *out);