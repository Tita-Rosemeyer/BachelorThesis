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

static Libel__setpoint setpin;
static Libel__state_t st;
static Libel__sensor_data sens;
static Libel__controller_pid_mem controller_mem;
static Libel__controller_pid_out controller_out;

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

void libel_from_baro(const baro_t *in, Libel__baro *out) {
    out->pressure = in->pressure;
    out->temperature = in->temperature;
    out->asl = in->asl;
}

void libel_from_axis3f(const Axis3f *in, Libel__vec3 *out) {
    out->x = in->x;
    out->y = in->y;
    out->z = in->z;
}

void libel_from_sensors(const sensorData_t *in, Libel__sensor_data *out) {
    libel_from_axis3f(&in->acc, &out->acc);
    libel_from_axis3f(&in->gyro, &out->gyro);
    libel_from_axis3f(&in->mag, &out->mag);
    libel_from_baro(&in->baro, &out->baro);
}

void libel_to_control(const Libel__control *in, control_t *out) {
    out->roll = in->c_roll;
    out->pitch = in->c_pitch;
    out->yaw = in->c_yaw;
    out->thrust = in->c_thrust;
}

void controllerPidInit(void)
{
  Libel__controller_pid_reset(&controller_mem);
}

bool controllerPidTest(void)
{
  bool pass = true;

  /* pass &= attitudeControllerTest(); */

  return pass;
}

void controllerPid(control_t *control, setpoint_t *setpoint,
                   const sensorData_t *sensors,
                   const state_t *state,
                   const uint32_t tick)
{
    libel_from_setpoint(setpoint, &setpin);
    libel_from_sensors(sensors, &sens);
    libel_from_state(state, &st);
    Libel__controller_pid_step(sens, st, setpin, &controller_out, &controller_mem);
    libel_to_control(&controller_out.control, control);
}

/* /\** */
/*  * Logging variables for the command and reference signals for the */
/*  * altitude PID controller */
/*  *\/ */
/* LOG_GROUP_START(controller) */
/* /\** */
/*  * @brief Thrust command */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust) */
/* /\** */
/*  * @brief Roll command */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll) */
/* /\** */
/*  * @brief Pitch command */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch) */
/* /\** */
/*  * @brief yaw command */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw) */
/* /\** */
/*  * @brief Gyro roll measurement in radians */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, r_roll, &r_roll) */
/* /\** */
/*  * @brief Gyro pitch measurement in radians */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch) */
/* /\** */
/*  * @brief Yaw  measurement in radians */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw) */
/* /\** */
/*  * @brief Acceleration in the zaxis in G-force */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, accelz, &accelz) */
/* /\** */
/*  * @brief Thrust command without (tilt)compensation */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust) */
/* /\** */
/*  * @brief Desired roll setpoint */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll) */
/* /\** */
/*  * @brief Desired pitch setpoint */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch) */
/* /\** */
/*  * @brief Desired yaw setpoint */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw) */
/* /\** */
/*  * @brief Desired roll rate setpoint */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll) */
/* /\** */
/*  * @brief Desired pitch rate setpoint */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch) */
/* /\** */
/*  * @brief Desired yaw rate setpoint */
/*  *\/ */
/* LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw) */
/* LOG_GROUP_STOP(controller) */
