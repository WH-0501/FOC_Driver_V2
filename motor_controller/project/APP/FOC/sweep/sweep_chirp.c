#include "sweep_chirp.h"
#include "sweep_config.h"
#include <string.h>
/* Quarter-wave polynomial. The phase accumulator and window stay unchanged.
 * Odd Taylor polynomial through degree 11 on [0, pi/2]. Truncation error
 * <= (pi/2)^13/13! = 5.7e-8; evaluate with the hardware single-precision FPU.
 * No argument-reduction library or sine lookup table in the ISR. */
float sweep_sine_phase(uint32_t phase)
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
  float value = sweep_sine_phase(s->phase) - s->dc;
  if (envelope < 1.0f) value *= envelope * envelope * (3.0f - 2.0f * envelope);
  s->phase += s->step;
  s->step += s->delta;
  s->error += s->remainder;
  if (s->error >= length - 1u) { s->error -= length - 1u; ++s->step; }
  return value;
}

float sweep_chirp_frequency(const sweep_chirp_t *s, float fs)
{ return (float)s->step * (fs * (1.0f / 4294967296.0f)); }

/* Foreground normalization; no spectrum is computed on device. */
float sweep_chirp_prepare(sweep_chirp_t *s, float low, float high, float fs, uint32_t length)
{
  uint32_t last, difference, i;
  double sum = 0.0, weight = 0.0;
  float maximum = 0.0f;
  memset(s, 0, sizeof(*s));
  s->first_step = (uint32_t)((double)low * 4294967296.0 / fs + 0.5);
  last = (uint32_t)((double)high * 4294967296.0 / fs + 0.5);
  difference = last - s->first_step;
  s->delta = difference / (length - 1u);
  s->remainder = difference % (length - 1u);
  s->ramp_gain = (1000.0f / SWEEP_CHIRP_EDGE_MS) / fs;
  sweep_chirp_reset(s);
  for (i = 0; i < length; ++i) {
    uint32_t edge = i < length-1u-i ? i : length-1u-i;
    float u = (float)edge * s->ramp_gain;
    weight += u < 1.0f ? u*u*(3.0f-2.0f*u) : 1.0f;
    sum += sweep_chirp_step(s, length, i);
  }
  s->dc = (float)(sum / weight); /* zero mean, without nonzero endpoint steps */
  sweep_chirp_reset(s);
  for (i = 0; i < length; ++i) {
    float x = sweep_chirp_step(s, length, i);
    if (x < 0.0f) x = -x;
    if (x > maximum) maximum = x;
  }
  sweep_chirp_reset(s);
  return maximum;
}
