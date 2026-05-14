#include "board.h"
#include "foc.h"

void board_init(void)
{
  current_hw_init();
}

void board_deinit(void)
{
  current_hw_deinit();
}

void board_apply_phase_current(motor_state_t *state)
{
  uint32_t i;
  const float kv = BOARD_ADC_VOLTS_PER_LSB;
  const float gain = BOARD_SO_AMPS_PER_VOLT;

  for (i = 0; i < 3; i++)
  {
    float v_pin;
    if (state->current_calibrating != 0U)
    {
      v_pin = (float)state->phase_current.adc_raw[i] * kv;
      state->phase_current.ampere[i] = (v_pin - BOARD_SO_BIAS_V) * gain;
    }
    else
    {
      v_pin = (float)((int32_t)state->phase_current.adc_raw[i] -
                      (int32_t)state->phase_current.adc_offset[i]) * kv;
      state->phase_current.ampere[i] = v_pin * gain;
    }
  }

  state->board_temp.value =
      (float)state->board_temp.adc_raw * BOARD_ADC_VREF_V / BOARD_ADC_FULL_SCALE;
}

void board_current_offset_cal_fsm_step(motor_handle_t *m)
{
  uint32_t i;
  static uint32_t s_cnt;
  static uint32_t s_sum[3];
  static uint8_t s_armed;

  if (m == NULL)
  {
    return;
  }

  if (m->fsm != STATE_CURRENT_CALIBRATION)
  {
    s_armed = 0U;
    return;
  }

  if (s_armed == 0U)
  {
    s_armed = 1U;
    s_cnt = 0U;
    s_sum[0] = 0U;
    s_sum[1] = 0U;
    s_sum[2] = 0U;
    m->state.current_calibrating = 1U;
  }

  for (i = 0U; i < 3U; i++)
  {
    s_sum[i] += (uint32_t)m->state.phase_current.adc_raw[i];
  }
  s_cnt++;

  if (s_cnt >= CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT)
  {
    for (i = 0U; i < 3U; i++)
    {
      m->state.phase_current.adc_offset[i] =
          (uint16_t)(s_sum[i] >> CURRENT_OFFSET_CALIBRATION_TIME);
    }
    m->state.current_calibrating = 0U;
    m->fsm = STATE_IDLE;
    s_armed = 0U;
  }
}
