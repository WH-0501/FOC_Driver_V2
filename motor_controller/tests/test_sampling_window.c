#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "at32_foc_timing.h"
#include "foc_modulation.h"

static unsigned long cases;

static void verify(float ud, float uq, float angle, float vbus, float duty_max)
{
  foc_modulation_t result;
  float alpha, beta, d, q;
  float tolerance = 8e-6f * vbus + 1e-6f;
  float duties[3];
  unsigned int i;

  assert(foc_modulate_voltage(ud, uq, angle, vbus, duty_max, &result));
  duties[0] = result.duty_a;
  duties[1] = result.duty_b;
  duties[2] = result.duty_c;
  for (i = 0; i < 3; ++i)
  {
    unsigned int ccr = (unsigned int)(duties[i] * AT32_FOC_TIMER_ARR);
    assert(isfinite(duties[i]));
    assert(duties[i] >= 0.0f && duties[i] <= duty_max);
    if (duty_max == AT32_FOC_CURRENT_MAX_DUTY)
    {
      assert(ccr <= AT32_FOC_CURRENT_MAX_CCR);
      /* Every bridge is low throughout the complete ADC sequence, with
       * the configured settling allowance on BOTH sides of the sequence. */
      assert(AT32_FOC_ADC_TRIGGER_CCR >= ccr + AT32_FOC_SETTLING_TICKS);
      assert(2u * AT32_FOC_TIMER_ARR - ccr >=
             AT32_FOC_ADC_TRIGGER_CCR + AT32_FOC_ADC_SEQUENCE_TICKS + AT32_FOC_SETTLING_TICKS);
    }
  }
  assert(fabsf((result.duty_a - result.duty_b) * vbus - (result.u_a - result.u_b)) < tolerance);
  assert(fabsf((result.duty_b - result.duty_c) * vbus - (result.u_b - result.u_c)) < tolerance);
  /* Reconstruct the dq voltage from differential duties, independent of the
   * modulation implementation, to detect phase-wise clipping or angle errors. */
  alpha = (2.0f * result.duty_a - result.duty_b - result.duty_c) * vbus / 3.0f;
  beta = (result.duty_b - result.duty_c) * vbus / sqrtf(3.0f);
  d = alpha * cosf(angle) + beta * sinf(angle);
  q = -alpha * sinf(angle) + beta * cosf(angle);
  assert(fabsf(d - ud * result.voltage_scale) < tolerance);
  assert(fabsf(q - uq * result.voltage_scale) < tolerance);
  assert(hypotf(d, q) <= vbus * duty_max / sqrtf(3.0f) + tolerance);
  assert(result.voltage_scale > 0.0f && result.voltage_scale <= 1.0f);
  ++cases;
}

static void invalid_inputs(void)
{
  foc_modulation_t before, output;
  memset(&before, 0x5a, sizeof(before));
  output = before;
  assert(!foc_modulate_voltage(NAN, 1, 0, 24, .8f, &output));
  assert(!foc_modulate_voltage(1, INFINITY, 0, 24, .8f, &output));
  assert(!foc_modulate_voltage(1, 1, NAN, 24, .8f, &output));
  assert(!foc_modulate_voltage(1, 1, 0, 0, .8f, &output));
  assert(!foc_modulate_voltage(1, 1, 0, -24, .8f, &output));
  assert(!foc_modulate_voltage(1, 1, 0, NAN, .8f, &output));
  assert(!foc_modulate_voltage(1, 1, 0, INFINITY, .8f, &output));
  assert(!foc_modulate_voltage(1, 1, 0, 24, 0, &output));
  assert(!foc_modulate_voltage(1, 1, 0, 24, NAN, &output));
  assert(!foc_modulate_voltage(1, 1, 0, 24, 1.01f, &output));
  assert(!foc_modulate_voltage(1, 1, 0, 24, .8f, NULL));
  assert(memcmp(&before, &output, sizeof(output)) == 0);
}

int main(void)
{
  const float buses[] = { 9.0f, 12.0f, 24.0f, 28.0f };
  const float limits[] = { AT32_FOC_CURRENT_MAX_DUTY, 1.0f };
  const float fractions[] = { -2.0f, -1.0f, -.5f, 0.0f, .5f, 1.0f, 2.0f };
  unsigned int b, l, d, q, angle;
  foc_modulation_t zero, huge;

  assert(AT32_FOC_ADC_SEQUENCE_TICKS == 321u);
  assert(AT32_FOC_SETTLING_TICKS == 180u);
  assert(AT32_FOC_ADC_TRIGGER_CCR == 2088u);
  assert(AT32_FOC_CURRENT_MAX_CCR == 1908u);
  for (b = 0; b < sizeof(buses) / sizeof(buses[0]); ++b)
    for (l = 0; l < sizeof(limits) / sizeof(limits[0]); ++l)
      for (d = 0; d < sizeof(fractions) / sizeof(fractions[0]); ++d)
        for (q = 0; q < sizeof(fractions) / sizeof(fractions[0]); ++q)
          for (angle = 0; angle < 720; ++angle)
            verify(fractions[d] * buses[b], fractions[q] * buses[b],
                   (float)angle * _2PI / 720.0f, buses[b], limits[l]);

  assert(foc_modulate_voltage(0, 0, 0, 24, AT32_FOC_CURRENT_MAX_DUTY, &zero));
  assert(zero.duty_a == zero.duty_b && zero.duty_b == zero.duty_c);
  assert(zero.duty_a == AT32_FOC_CURRENT_MAX_DUTY * .5f);
  assert(foc_modulate_voltage(FLT_MAX, -FLT_MAX, 1, 24, AT32_FOC_CURRENT_MAX_DUTY, &huge));
  assert(isfinite(huge.duty_a) && isfinite(huge.duty_b) && isfinite(huge.duty_c));
  invalid_inputs();
  printf("PASS: %lu voltage/angle/bus cases; invalid inputs; zero vector; large inputs\n", cases);
  printf("ADC budget=%llu ticks, trigger=%llu, CCR max=%llu, duty max=%.8f\n",
         (unsigned long long)AT32_FOC_ADC_SEQUENCE_TICKS,
         (unsigned long long)AT32_FOC_ADC_TRIGGER_CCR,
         (unsigned long long)AT32_FOC_CURRENT_MAX_CCR, (double)AT32_FOC_CURRENT_MAX_DUTY);
  return 0;
}
