#ifndef SWEEP_CHIRP_H
#define SWEEP_CHIRP_H
#include <stdint.h>
/* Stateless hardware interface: one caller-owned periodic linear DDS chirp.
 * Integer phase + remainder accumulation avoid long float oscillator drift.
 * 32 bytes, no samples buffer; bounded sine polynomial, no lookup table. */
typedef struct {
  uint32_t phase, step, first_step, delta, remainder, error;
  float ramp_gain, dc;
} sweep_chirp_t;
float sweep_sine_phase(uint32_t phase);
float sweep_chirp_prepare(sweep_chirp_t *s, float low, float high, float fs, uint32_t length);
void sweep_chirp_reset(sweep_chirp_t *s);
float sweep_chirp_step(sweep_chirp_t *s, uint32_t length, uint32_t index);
float sweep_chirp_frequency(const sweep_chirp_t *s, float fs);
#endif
