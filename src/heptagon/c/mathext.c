#include <math.h>
#include "mathext_types.h"

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

