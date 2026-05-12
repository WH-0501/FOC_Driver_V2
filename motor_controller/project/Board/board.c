#include "board.h"
#include "foc.h"

typedef struct
{
  uint16_t cnt;
  uint32_t sum[3];
} offset_accum_t;

static offset_accum_t s_offset_accum;

static void offset_accum_reset(void)
{
  s_offset_accum.cnt = 0;
  s_offset_accum.sum[0] = 0;
  s_offset_accum.sum[1] = 0;
  s_offset_accum.sum[2] = 0;
}

void board_init(void)
{
  offset_accum_reset();
}

void board_apply_phase_current(motor_state_t *state)
{
  uint32_t i;
  const float kv = BOARD_ADC_VOLTS_PER_LSB;
  const float gain = BOARD_SO_AMPS_PER_VOLT;

  for (i = 0; i < 3; i++)
  {
    float v_pin;
    if (state->current_calibrating)
    {
      v_pin = (float)state->m_phase_current.adc_raw[i] * kv;
      state->m_phase_current.ampere[i] = (v_pin - BOARD_SO_BIAS_V) * gain;
    }
    else
    {
      v_pin = (float)((int32_t)state->m_phase_current.adc_raw[i] -
                      (int32_t)state->m_phase_current.adc_offset[i]) *
              kv;
      state->m_phase_current.ampere[i] = v_pin * gain;
    }
  }

  state->board_temp.value =
      (float)state->board_temp.adc_raw * BOARD_ADC_VREF_V / BOARD_ADC_FULL_SCALE;
}

void board_current_offset_calculate(void)
{
  uint32_t i;

  if (!g_motor.state.current_calibrating)
  {
    return;
  }

  for (i = 0; i < 3; i++)
  {
    s_offset_accum.sum[i] += (uint32_t)g_motor.state.m_phase_current.adc_raw[i];
  }

  s_offset_accum.cnt++;

  if (s_offset_accum.cnt >= CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT)
  {
    for (i = 0; i < 3; i++)
    {
      g_motor.state.m_phase_current.adc_offset[i] =
          (uint16_t)(s_offset_accum.sum[i] >> CURRENT_OFFSET_CALIBRATION_TIME);
    }
    offset_accum_reset();
  }
}
