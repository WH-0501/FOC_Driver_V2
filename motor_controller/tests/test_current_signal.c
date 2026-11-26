#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "current_sense.h"
#include "foc_math.h"

int main(void)
{
  motor_handle_t m;
  unsigned i, j, cases = 0;
  float alpha, beta, d, q;
  memset(&m, 0, sizeof(m));
  m.current_offset_cal_done = 1;
  /* Independent double-precision reference: verify signed subtraction at
   * both ADC endpoints and every 12-bit count, including below offset. */
  for (i = 0; i < 4096; ++i)
  {
    for (j = 0; j < 3; ++j)
    {
      m.state.phase_current.adc_raw[j] = (uint16_t)i;
      m.state.phase_current.adc_offset[j] = (uint16_t)(2047 + j);
    }
    current_sense_process_sample(&m);
    for (j = 0; j < 3; ++j)
    {
      double expected = ((int)i - (int)(2047+j)) * (double)CURRENT_SENSE_ADC_VOLTS_PER_LSB * (double)CURRENT_SENSE_SO_GAIN;
      assert(fabs(m.state.phase_current.ampere[j] - expected) < 1e-6);
      assert(m.state.phase_current.adc_raw[j] == i);
    }
  }
  /* A shared ADC/sense offset must not become torque feedback at any angle.
   * Balanced currents retain unit gain; arbitrary-angle Park preserves norm. */
  for (i = 0; i < 8192; ++i)
  {
    float theta = (float)i * _2PI / 8192.0f;
    float ia = .05f*cosf(theta), ib = .05f*cosf(theta-_2PI/3), ic = .05f*cosf(theta+_2PI/3);
    for (j = 0; j < 5; ++j)
    {
      float common = ((int)j-2)*.2f;
      clarke_transform(ia+common, ib+common, ic+common, &alpha, &beta);
      assert(fabsf(alpha - ia) < 1e-7f);
      assert(fabsf(beta - .05f*sinf(theta)) < 1e-7f);
      park_transform(alpha, beta, theta, &d, &q);
      assert(fabsf(d-.05f) < 2e-6f && fabsf(q) < 2e-6f);
      ++cases;
    }
  }
  /* Zero and exact negative whole turns used to return 2*pi. */
  for (i = 0; i <= 128; ++i)
  {
    float theta = ((int)i-64)*_2PI;
    float a = angle_normalize(theta);
    assert(a >= 0 && a < _2PI);
    assert(fabsf(sinf(a)-sinf(theta)) < 3e-5f);
  }
  assert(angle_normalize(0) == 0);
  for (i = 0; i < 200000; ++i)
  {
    float theta = ((int)i-100000)*.001f;
    float a = angle_normalize(theta);
    assert(a >= 0 && a < _2PI);
    assert(fabsf(sinf(a)-sinf(theta)) < 1e-5f);
  }
  puts("PASS: 4096 ADC codes x 3 offsets; 40960 balanced/common-mode dq cases; 200129 angle wraps");
  return 0;
}
