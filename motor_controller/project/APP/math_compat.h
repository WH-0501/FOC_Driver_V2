#ifndef __MATH_COMPAT_H__
#define __MATH_COMPAT_H__

#include <math.h>
#include "compiler_port.h"

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define FMAX(a, b) ((a) > (b) ? (a) : (b))
#define FMIN(a, b) ((a) < (b) ? (a) : (b))

#define SQ(x) ((x) * (x))

/* 以最大步长 step 将 current 向 target 逼近 */
APP_STATIC_INLINE float ramp_toward(float current, float target, float step)
{
  float delta;

  if (step <= 0.0f)
  {
    return current;
  }

  delta = target - current;
  if (delta > step)
  {
    return current + step;
  }
  if (delta < -step)
  {
    return current - step;
  }
  return target;
}

#endif /* __MATH_COMPAT_H__ */
