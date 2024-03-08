#include "tita_debug_functions.h"
// Check if states are equal

int equal_floats(char *name, float f1, float f2) {
    float diff = fabsf(f1 - f2);
    if (diff > EPS) {
    // if (f1 != f2) {
        DEBUG_PRINT("Mismatch in %s!  expected: %.4f got: %.4f\n", name, (double) f1, (double)f2);
        //DEBUG_PRINT("Difference is %.2f\n", diff);
        return 0;
    }
    //DEBUG_PRINT("Good in %s\n", name);//DEBUG_PRINT("Good in %s : expected %.2f, got %.2f\n", name, f1, f2);
    return 1;
}

int equal_axis3f(char *varname, Axis3f ref, Axis3f got) {
  float dx = fabsf(ref.x - got.x);
  float dy = fabsf(ref.y - got.y);
  float dz = fabsf(ref.z - got.z);
  if(fmaxf(dx, fmaxf(dy,dz))> EPS){
    DEBUG_PRINT("Mismatch in %s! expected: (%.4f,%.4f,%.4f) got: (%.4f,%.4f,%.4f)\n", varname,(double) ref.x,(double) ref.y,(double) ref.z, (double) got.x,(double) got.y,(double) got.z);
    return false;
  }
  return true;
}

int equal_int16(const char *varname, int16_t i1, int16_t i2) {
    int diff = fabsf(i1 - i2);
    if (diff > fabsf(i1 * 0.1)) {
        DEBUG_PRINT("Divergence in %s : expected %d, got %d\n", varname,  i1,  i2);
        return false;
    }
    //DEBUG_PRINT("Good in %s : expected %d, got %d\n", varname, i1, i2);
    return true;
}

int equal_attitude(const attitude_t *attitude1, const attitude_t *attitude2) {
    int roll = equal_floats("attitude.roll", attitude1->roll, attitude2->roll);
    int pitch = equal_floats("attitude.pitch", attitude1->pitch, attitude2->pitch);
    int yaw = equal_floats("attitude.yaw", attitude1->yaw, attitude2->yaw);
    return roll && pitch && yaw;
}

int equal_attitudeQuaternion(const quaternion_t *attitudeQuaternion1, const quaternion_t *attitudeQuaternion2) {
    int w = equal_floats("attitudeQuaternion.w", attitudeQuaternion1->w, attitudeQuaternion2->w);
    int x = equal_floats("attitudeQuaternion.x", attitudeQuaternion1->x, attitudeQuaternion2->x);
    int y = equal_floats("attitudeQuaternion.y", attitudeQuaternion1->y, attitudeQuaternion2->y);
    int z = equal_floats("attitudeQuaternion.z", attitudeQuaternion1->z, attitudeQuaternion2->z);
    return x && y && z && w;
}

int equal_position(const point_t *position1, const point_t *position2) {
    int x = equal_floats("position.x", position1->x, position2->x);
    int y = equal_floats("position.y", position1->y, position2->y);
    int z = equal_floats("position.z", position1->z, position2->z);
    return x && y && z;
}

int equal_velocity(const velocity_t *velocity1, const velocity_t *velocity2) {
    int x = equal_floats("velocity.x", velocity1->x, velocity2->x);
    int y = equal_floats("velocity.y", velocity1->y, velocity2->y);
    int z = equal_floats("velocity.z", velocity1->z, velocity2->z);
    return x && y && z;
}

int equal_acc(const acc_t *acc1, const acc_t *acc2) {
    int x = equal_floats("acc.x", acc1->x, acc2->x);
    int y = equal_floats("acc.y", acc1->y, acc2->y);
    int z = equal_floats("acc.z", acc1->z, acc2->z);
    return x && y && z;
}

void debug_print_state(state_t *struc){
    DEBUG_PRINT("yaw: %.2f ", (double)struc->attitude.yaw);
    DEBUG_PRINT("pitch: %.2f ", (double)struc->attitude.pitch);
    DEBUG_PRINT("roll: %.2f ", (double)struc->attitude.roll);
    DEBUG_PRINT("q: (%.2f, %.2f, %.2f, %.2f) ", (double)struc->attitudeQuaternion.w, (double) struc->attitudeQuaternion.x, (double)struc->attitudeQuaternion.y, (double)struc->attitudeQuaternion.z);
    DEBUG_PRINT("vel: (%.2f, %.2f, %.2f) ",(double) struc->velocity.x, (double)struc->velocity.y,(double) struc->velocity.z);
    DEBUG_PRINT("pos: (%.2f, %.2f, %.2f) ", (double)struc->position.x, (double)struc->position.y,(double) struc->position.z);
    DEBUG_PRINT("acc: (%.2f, %.2f, %.2f) ", (double)struc->acc.x, (double)struc->acc.y, (double)struc->acc.z);
}

int equal_state(state_t *state1, state_t *state2){
    int quat = equal_attitudeQuaternion(&state1->attitudeQuaternion, &state2->attitudeQuaternion);
    int attitude = equal_attitude(&state1->attitude, &state2->attitude);
    int pos = equal_position(&state1->position, &state2->position);
    int vel = equal_velocity(&state1->velocity, &state2->velocity);
    int acc = equal_acc(&state1->acc, &state2->acc);
    // if (pos && vel && acc)
    if (attitude && quat && pos && vel && acc)
        return 1;
    DEBUG_PRINT("in ExternalizeState \n");
    // debug_print_state(state2);
    return 0;
}
int equal_controls(const control_t *control1, const control_t *control2) {
    bool a = equal_int16("control.roll", control1->roll, control2->roll);
    bool b = equal_int16("control.pitch", control1->pitch, control2->pitch);
    bool c = equal_int16("control.yaw", control1->yaw, control2->yaw);
    bool d = equal_floats("control.thrust", control1->thrust, control2->thrust);
    return
        a && b && c && d;
}

int equal_float_array(const char* varname, float* ref, float *got, int n){
  for (int i = 0; i < n; i++)
  {
    if(fabsf(ref[i] - got[i])>EPS){
      DEBUG_PRINT("Mismatch in %s (index %i/%i)! expected: %.5f got: %.5f (diff: %.4f) \n", varname, i, n, (double) ref[i], (double) got[i],(double) fabsf(ref[i] - got[i]));
      //print_statematrix(ref, got);
      return false;
    }
  }
  //DEBUG_PRINT("Good in %s \n", varname);
  return true;
}

int equal_arm_matrix_f32(const char* varname, arm_matrix_instance_f32 ref, arm_matrix_instance_f32 got){
  if(ref.numCols != got.numCols || ref.numRows != got.numRows){
    DEBUG_PRINT("%s shape mismatch! expected: (%i,%i) got: (%i,%i)\n", varname, ref.numRows, ref.numCols, got.numRows, got.numCols);
    return false;
  }
  return equal_float_array(varname, ref.pData, got.pData, ref.numCols*ref.numRows);
  
}

int equal_coredata(const char* debug_name, kalmanCoreData_t *refcore, kalmanCoreData_t* gotcore){
  
  int equal_quat = equal_float_array("initial Quaternion", refcore->initialQuaternion, gotcore->initialQuaternion, 4);
  int equal_ref_height =  equal_floats("baroReferenceHeight", refcore->baroReferenceHeight, gotcore->baroReferenceHeight);
  int equal_covariance =  equal_arm_matrix_f32("Covariance Matrix", refcore->Pm, gotcore->Pm);
  int equal_attitude =  equal_float_array("Attitude quaternion", refcore->q, gotcore->q, 4);
  int equal_R = equal_float_array("R", refcore->R, gotcore->R, 9);
  int equal_state = equal_float_array("State array", refcore->S, gotcore->S, 9);
  int res =  equal_R && equal_quat && equal_ref_height && equal_covariance && equal_state && equal_attitude;
  if(!res)
    DEBUG_PRINT("In %s\n", debug_name);
  return res;
}


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

void libel_from_quaternion_kalman(float in[4], Mathext__quaternion *out) {
  out->qw = in[0];
  out->qx = in[1];
  out->qy = in[2];
  out->qz = in[3];
}

void libel_to_quaternion_kalman(Mathext__quaternion *in, float out[4]) {
  out[0] = in->qw;
  out[1] = in->qx;
  out[2] = in->qy;
  out[3] = in->qz;
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

void libel_array_to_quadrocopter(float p_array[9], Mathext__quadrocopter_state* q) {
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
  libel_array_to_quadrocopter(p_array[0], &p->kc_state_X);
  libel_array_to_quadrocopter(p_array[1], &p->kc_state_Y);
  libel_array_to_quadrocopter(p_array[2], &p->kc_state_Z);
  libel_array_to_quadrocopter(p_array[3], &p->kc_state_PX);
  libel_array_to_quadrocopter(p_array[4], &p->kc_state_PY);
  libel_array_to_quadrocopter(p_array[5], &p->kc_state_PZ);
  libel_array_to_quadrocopter(p_array[6], &p->kc_state_D0);
  libel_array_to_quadrocopter(p_array[7], &p->kc_state_D1);
  libel_array_to_quadrocopter(p_array[8], &p->kc_state_D2);
}

void libel_from_coreData(kalmanCoreData_t *in, Mathext__kalman_coredata_t *out){
  libel_array_to_quadrocopter(in->S, &out->s);
  libel_from_quaternion_kalman(in->q, &out->q);
  memcpy( out->r, in->R,sizeof(float)*3*3);
  libel_matrix_to_covariance_matrix(in->P, &out->p);
  libel_from_quaternion_kalman(in->initialQuaternion, &out->initial_quaternion);
}

void libel_to_coreData(Mathext__kalman_coredata_t *in, kalmanCoreData_t *out){
  libel_quadrocopter_to_array(&in->s, out->S);
  libel_to_quaternion_kalman(&in->q, out->q);
  memcpy(out->R, in->r,  sizeof(float)*3*3);
  libel_covariance_matrix_to_matrix(&in->p, out->P);
  out->Pm.numCols = 9;
  out->Pm.numRows = 9;
  out->Pm.pData = (float *) out->P;
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

void libel_to_quaternion2_kalman(Mathext__quaternion *in, quaternion_t *out) {
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


void libel_to_tof(Libel__tof *in , tofMeasurement_t *out){
  out->distance = in->distance;
  out->stdDev = in->stdDev;
  out->timestamp  = in->timestamp;
}
void libel_from_tof(tofMeasurement_t *in , Libel__tof *out){
  out->distance = in->distance;
  out->stdDev = in->stdDev;
  out->timestamp = in->timestamp;

}