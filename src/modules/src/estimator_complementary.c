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

#include "estimator_lib.h"

static Axis3f gyro;
static Axis3f acc;
static baro_t baro;
static tofMeasurement_t tof;

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

void estimatorComplementaryInit(void)
{
  sensfusion6Init();
}

bool estimatorComplementaryTest(void)
{
  bool pass = true;

  pass &= sensfusion6Test();

  return pass;
}

static Estimator__state_t st;
static Estimator__sensor_data sens;
static Estimator__estimator_complementary_mem estimator_complementary_mem;
static Estimator__estimator_complementary_out estimator_complementary_out;

void libel_from_vec3(const struct vec3_s *in, Estimator__vec3 *out) {
    out->x = in->x;
    out->y = in->y;
    out->z = in->z;
}

void libel_to_vec3(const Estimator__vec3 *in, struct vec3_s *out) {
    out->x = in->x;
    out->y = in->y;
    out->z = in->z;
}

void libel_from_attitude(const attitude_t *in, Estimator__attitude *out) {
    out->roll = in->roll;
    out->pitch = in->pitch;
    out->yaw = in->yaw;
}

void libel_to_attitude(const Estimator__attitude *in, attitude_t *out) {
    out->roll = in->roll;
    out->pitch = in->pitch;
    out->yaw = in->yaw;
}

void libel_from_quaternion(const quaternion_t *in, Estimator__quaternion *out) {
    out->qx = in->x;
    out->qy = in->y;
    out->qz = in->z;
    out->qw = in->w;
}

void libel_to_quaternion(const Estimator__quaternion *in, quaternion_t *out) {
    out->x = in->qx;
    out->y = in->qy;
    out->z = in->qz;
    out->w = in->qw;
}

void libel_to_state(Estimator__state_t *in, const state_t *out) {
    libel_to_attitude(&in->st_attitude, &out->attitude);
    libel_to_quaternion(&in->st_attitude_quat, &out->attitudeQuaternion);
    libel_to_vec3(&in->st_position, &out->position);
    libel_to_vec3(&in->st_velocity, &out->velocity);
    libel_to_vec3(&in->st_acc, &out->acc);
}

void libel_from_baro(const baro_t *in, Estimator__baro *out) {
    out->pressure = in->pressure;
    out->temperature = in->temperature;
    out->asl = in->asl;
}

void libel_from_tof(const tofMeasurement_t *in, Estimator__tof *out) {
    out->timestamp = in->timestamp;
    out->distance = in->distance;
    out->stdDev = in->stdDev;
}

void libel_from_sensors(const Axis3f *acc, const Axis3f *gyro, const baro_t *baro, 
                        const tofMeasurement_t *tof, Estimator__sensor_data *out) {
    libel_from_axis3f(acc, &out->acc);
    libel_from_axis3f(gyro, &out->gyro);
    libel_from_tof(tof, &out->tof);
    libel_from_baro(baro, &out->baro);
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

  libel_from_sensors(&acc, &gyro, &baro, &tof, &sens);
  Estimator__estimator_complementary_step(sens, &estimator_complementary_out, &estimator_complementary_mem);
  libel_to_state(&estimator_complementary_out.st, state);

}
