#define DEBUG_MODULE "CONTROLLER_PID"
#include "debug.h"

#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#include "libel.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

static Libel__setpoint setpin;
static Libel__state_t st;
static Libel__controller_pid1_mem controller1_mem;
static Libel__controller_pid1_out controller1_out;

void libel_from_vec3(const struct vec3_s *in, Libel__vec3 *out) {
    out->x = in->x;
    out->y = in->y;
    out->z = in->z;
}

void libel_to_vec3(const Libel__vec3 *in, struct vec3_s *out) {
    out->x = in->x;
    out->y = in->y;
    out->z = in->z;
}

void libel_from_attitude(const attitude_t *in, Libel__attitude *out) {
    out->roll = in->roll;
    out->pitch = in->pitch;
    out->yaw = in->yaw;
}

void libel_to_attitude(const Libel__attitude *in, attitude_t *out) {
    out->roll = in->roll;
    out->pitch = in->pitch;
    out->yaw = in->yaw;
}

void libel_from_quaternion(const quaternion_t *in, Libel__quaternion *out) {
    out->qx = in->x;
    out->qy = in->y;
    out->qz = in->z;
    out->qw = in->w;
}

void libel_to_quaternion(const Libel__quaternion *in, quaternion_t *out) {
    out->x = in->qx;
    out->y = in->qy;
    out->z = in->qz;
    out->w = in->qw;
}

Libel__stab_mode libel_from_mode(enum mode_e m) {
    switch(m) {
        case modeDisable:
            return Libel__Disable;
        case modeAbs:
            return Libel__Abs;
        default:
            return Libel__Velocity;
    }
}

enum mode_e libel_to_mode(Libel__stab_mode m) {
    switch(m) {
        case Libel__Disable:
            return modeDisable;
        case Libel__Abs:
            return modeAbs;
        default:
            return modeVelocity;
    }
}

void libel_from_setpoint(setpoint_t *in, Libel__setpoint *out) {
    libel_from_attitude(&in->attitude, &out->attitude);
    libel_from_attitude(&in->attitudeRate, &out->attitude_rate);
    libel_from_quaternion(&in->attitudeQuaternion, &out->attitude_quat);
    out->thrust = in->thrust;
    libel_from_vec3(&in->position, &out->position);
    libel_from_vec3(&in->velocity, &out->velocity);
    libel_from_vec3(&in->acceleration, &out->acceleration);
    out->velocity_body = in->velocity_body;
    out->mode.mx = libel_from_mode(in->mode.x);
    out->mode.my = libel_from_mode(in->mode.y);
    out->mode.mz = libel_from_mode(in->mode.z);
    out->mode.mroll = libel_from_mode(in->mode.roll);
    out->mode.mpitch = libel_from_mode(in->mode.pitch);
    out->mode.myaw = libel_from_mode(in->mode.yaw);
    out->mode.mquat = libel_from_mode(in->mode.quat);
}

void libel_to_setpoint(Libel__setpoint *in, setpoint_t *out) {
    libel_to_attitude(&in->attitude, &out->attitude);
    libel_to_attitude(&in->attitude_rate, &out->attitudeRate);
    libel_to_quaternion(&in->attitude_quat, &out->attitudeQuaternion);
    out->thrust = in->thrust;
    libel_to_vec3(&in->position, &out->position);
    libel_to_vec3(&in->velocity, &out->velocity);
    libel_to_vec3(&in->acceleration, &out->acceleration);
    out->velocity_body = in->velocity_body;
    out->mode.x = libel_to_mode(in->mode.mx);
    out->mode.y = libel_to_mode(in->mode.my);
    out->mode.z = libel_to_mode(in->mode.mz);
    out->mode.roll = libel_to_mode(in->mode.mroll);
    out->mode.pitch = libel_to_mode(in->mode.mpitch);
    out->mode.yaw = libel_to_mode(in->mode.myaw);
    out->mode.quat = libel_to_mode(in->mode.mquat);
}

void libel_from_state(const state_t *in, Libel__state_t *out) {
    libel_from_attitude(&in->attitude, &out->st_attitude);
    libel_from_quaternion(&in->attitudeQuaternion, &out->st_attitude_quat);
    libel_from_vec3(&in->position, &out->st_position);
    libel_from_vec3(&in->velocity, &out->st_velocity);
    libel_from_vec3(&in->acc, &out->st_acc);
}

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  /* positionControllerInit(); */
  Libel__controller_pid1_reset(&controller1_mem);
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

/* static float capAngle(float angle) { */
/*   float result = angle; */

/*   while (result > 180.0f) { */
/*     result -= 360.0f; */
/*   } */

/*   while (result < -180.0f) { */
/*     result += 360.0f; */
/*   } */

/*   return result; */
/* } */

void controllerPid(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  /* // Une fois tous les 2 */
  /* if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) { */
  /*   // Rate-controled YAW is moving YAW angle setpoint */
  /*   if (setpoint->mode.yaw == modeVelocity) { */
  /*     attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT); */

  /*     #ifdef YAW_MAX_DELTA */
  /*     float delta = capAngle(attitudeDesired.yaw-state->attitude.yaw); */
  /*     // keep the yaw setpoint within +/- YAW_MAX_DELTA from the current yaw */
  /*       if (delta > YAW_MAX_DELTA) */
  /*       { */
  /*         attitudeDesired.yaw = state->attitude.yaw + YAW_MAX_DELTA; */
  /*       } */
  /*       else if (delta < -YAW_MAX_DELTA) */
  /*       { */
  /*         attitudeDesired.yaw = state->attitude.yaw - YAW_MAX_DELTA; */
  /*       } */
  /*     #endif */
  /*   } else { */
  /*     attitudeDesired.yaw = setpoint->attitude.yaw; */
  /*   } */

  /*   attitudeDesired.yaw = capAngle(attitudeDesired.yaw); */
  /* } */

  /* // Une fois tous les 10 */
  /* if (RATE_DO_EXECUTE(POSITION_RATE, tick)) { */
  /*   positionController(&actuatorThrust, &attitudeDesired, setpoint, state); */
  /* } */

  libel_from_setpoint(setpoint, &setpin);
  libel_from_state(state, &st);
  Libel__controller_pid1_step(setpin, st, &controller1_out, &controller1_mem);
  actuatorThrust = controller1_out.thrust;
  libel_to_attitude(&controller1_out.att_desired, &attitudeDesired);

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    /* // Switch between manual and automatic position control */
    /* if (setpoint->mode.z == modeDisable) { */
    /*   actuatorThrust = setpoint->thrust; */
    /* } */
    /* if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) { */
    /*   attitudeDesired.roll = setpoint->attitude.roll; */
    /*   attitudeDesired.pitch = setpoint->attitude.pitch; */
    /* } */

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    Libel__controller_pid1_reset(&controller1_mem);
    /* positionControllerResetAllPID(); */

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

