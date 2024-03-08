/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * estimator_complementary.c - a complementary estimator
 */

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "stabilizer.h"
#include "estimator_complementary.h"
#include "sensfusion6.h"
#include "position_estimator.h"
#include "sensors.h"
#include "stabilizer_types.h"
#include "static_mem.h"

#include "log.h"
#include "libel.h"

static Axis3f gyro;
static Axis3f acc;
static baro_t baro;
static tofMeasurement_t tof;

static Libel__sensor_data_est sens;
static Libel__estimator_complementary_mem estimator_complementary_mem;
static Libel__estimator_complementary_out estimator_complementary_out;

void estimatorComplementaryInit(void)
{
  Libel__estimator_complementary_reset(&estimator_complementary_mem);
  sensfusion6Init();
}

bool estimatorComplementaryTest(void)
{
  bool pass = true;

  pass &= sensfusion6Test();

  return pass;
}

void libel_from_vec3_estimator(const struct vec3_s *in, Libel__vec3 *out) {
    out->x = in->x;
    out->y = in->y;
    out->z = in->z;
}

void libel_to_vec3_estimator(const Libel__vec3 *in, struct vec3_s *out) {
    out->x = in->x;
    out->y = in->y;
    out->z = in->z;
}

void libel_from_attitude_estimator(const attitude_t *in, Libel__attitude *out) {
    out->roll = in->roll;
    out->pitch = in->pitch;
    out->yaw = in->yaw;
}

void libel_to_attitude_estimator(const Libel__attitude *in, attitude_t *out) {
    out->roll = in->roll;
    out->pitch = in->pitch;
    out->yaw = in->yaw;
}

void libel_from_quaternion_estimator(const quaternion_t *in, Mathext__quaternion *out) {
    out->qx = in->x;
    out->qy = in->y;
    out->qz = in->z;
    out->qw = in->w;
}

void libel_to_quaternion_estimator(const Mathext__quaternion *in, quaternion_t *out) {
    out->x = in->qx;
    out->y = in->qy;
    out->z = in->qz;
    out->w = in->qw;
}

void libel_to_state_estimator(const Libel__state_t *in, state_t *out) {
    libel_to_attitude_estimator(&in->st_attitude, &out->attitude);
    libel_to_quaternion_estimator(&in->st_attitude_quat, &out->attitudeQuaternion);
    libel_to_vec3_estimator(&in->st_position, &out->position);
    libel_to_vec3_estimator(&in->st_velocity, &out->velocity);
    libel_to_vec3_estimator(&in->st_acc, &out->acc);
}

void libel_from_baro_estimator(const baro_t *in, Libel__baro *out) {
    out->pressure = in->pressure;
    out->temperature = in->temperature;
    out->asl = in->asl;
}

void libel_from_tof_estimator(const tofMeasurement_t *in, Libel__tof *out) {
    out->timestamp = in->timestamp;
    out->distance = in->distance;
    out->stdDev = in->stdDev;
}

void libel_from_axis3f_estimator(const Axis3f *in, Libel__vec3 *out) {
    out->x = in->x;
    out->y = in->y;
    out->z = in->z;
}

void libel_from_sensors_estimator(const Axis3f *acc, const Axis3f *gyro, const baro_t *baro, 
                        const tofMeasurement_t *tof, Libel__sensor_data_est *out) {
    libel_from_axis3f_estimator(acc, &out->acc_est);
    libel_from_axis3f_estimator(gyro, &out->gyro_est);
    libel_from_tof_estimator(tof, &out->tof);
    libel_from_baro_estimator(baro, &out->baro_est);
}

void estimatorComplementary(state_t *state, const uint32_t tick)
{
  // Pull the latest sensors values of interest; discard the rest
  measurement_t m;
  while (estimatorDequeue(&m)) {
    switch (m.type)
    {
    case MeasurementTypeGyroscope:
      gyro = m.data.gyroscope.gyro;
      break;
    case MeasurementTypeAcceleration:
      acc = m.data.acceleration.acc;
      break;
    case MeasurementTypeBarometer:
      baro = m.data.barometer.baro;
      break;
    case MeasurementTypeTOF:
      tof = m.data.tof;
      break;
    default:
      break;
    }
  }

  libel_from_sensors_estimator(&acc, &gyro, &baro, &tof, &sens);
  Libel__estimator_complementary_step(sens, &estimator_complementary_out, &estimator_complementary_mem);
  libel_to_state_estimator(&estimator_complementary_out.st, state);

}


LOG_GROUP_START(tof)

/**
 * @brief Distance 
 */
LOG_ADD_CORE(LOG_FLOAT, dista, &tof.distance)

/**
 * @brief Standard Deviation
 */
LOG_ADD_CORE(LOG_FLOAT, stdDev, &tof.stdDev)

/**
 * @brief timestamp (s)
 */
LOG_ADD_CORE(LOG_FLOAT, time, &tof.timestamp)
LOG_GROUP_STOP(acc)
