/* Check the actual private phase-to-sine function against double libm. */
#include "../project/APP/FOC/sweep/sweep_chirp.c"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static double max_error, sum_squared;
static unsigned count;
static void check_phase(uint32_t phase)
{
  double reference = sin((double)phase * (6.283185307179586476925286766559 / 4294967296.0));
  float value = sine_phase(phase);
  double error = fabs((double)value - reference);
  if (error > max_error) max_error = error;
  sum_squared += error * error;
  ++count;
  assert(isfinite(value) && fabsf(value) <= 1.00000024f);
}
int main(void)
{
  uint32_t i, random = 314159;
  for (i = 0; i < (1u << 22); ++i) check_phase(i << 10);
  for (i = 0; i < 1000000u; ++i) {
    random = 1664525u * random + 1013904223u;
    check_phase(random);
  }
  /* Quadrant boundaries and the previous interpolation-grid boundaries. */
  for (i = 0; i < 1024u; ++i) {
    int j;
    for (j = -64; j <= 64; ++j) check_phase((i << 22) + (uint32_t)j);
  }
  assert(max_error < 4e-7);
  assert(sqrt(sum_squared / count) < 6e-8);
  printf("PASS chirp sine: %u phases, max %.12g / RMS %.12g against double sin\n",
         count, max_error, sqrt(sum_squared / count));
  return 0;
}
