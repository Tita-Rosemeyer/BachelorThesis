#include <math.h>
#include "mathext_types.h"
#include "FreeRTOS.h"
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
