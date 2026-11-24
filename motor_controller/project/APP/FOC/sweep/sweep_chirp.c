#include "sweep_chirp.h"
/* Quarter-wave polynomial. The phase accumulator and window stay unchanged.
 * Odd Taylor polynomial through degree 11 on [0, pi/2]. Truncation error
 * <= (pi/2)^13/13! = 5.7e-8; evaluate with the hardware single-precision FPU.
 * No argument-reduction library or sine lookup table in the ISR. */
static float sine_phase(uint32_t phase)
{
  uint32_t q = phase >> 30, p = phase & 0x3fffffffu;
  float x, z, value;
  if (q & 1u) p = 0x40000000u - p;
  x = (float)p * (1.57079632679489661923f / 1073741824.0f);
  z = x * x;
  value = x * (1.0f + z * (-1.0f/6.0f + z * (1.0f/120.0f +
          z * (-1.0f/5040.0f + z * (1.0f/362880.0f + z * (-1.0f/39916800.0f))))));
  return q & 2u ? -value : value;
}

void sweep_chirp_reset(sweep_chirp_t *s)
{ s->phase = 0u; s->step = s->first_step; s->error = 0u; }
float sweep_chirp_step(sweep_chirp_t *s, uint32_t length, uint32_t index)
{
  uint32_t edge = index < length-1u-index ? index : length-1u-index;
  float envelope = (float)edge * s->ramp_gain;
  float value = sine_phase(s->phase) - s->dc;
  if (envelope < 1.0f) value *= envelope * envelope * (3.0f - 2.0f * envelope);
  s->phase += s->step;
  s->step += s->delta;
  s->error += s->remainder;
  if (s->error >= length - 1u) { s->error -= length - 1u; ++s->step; }
  return value;
}
float sweep_chirp_frequency(const sweep_chirp_t *s, float fs)
{ return (float)s->step * (fs * (1.0f / 4294967296.0f)); }
