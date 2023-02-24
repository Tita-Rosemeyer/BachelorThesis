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

#endif // MATHEXT_H
