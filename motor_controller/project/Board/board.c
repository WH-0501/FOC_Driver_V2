#include "board.h"

void board_init(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
  }

  current_hw_init();
  /* 须在 foc_init() 之后调用：g_motor 已 memset + 标志位就绪，且 memset 不会覆盖本段写回的 offset */
  board_current_offset_calibration(m);
}

void board_deinit(void)
{
  current_hw_deinit();
}

void board_current_offset_calibration(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
  }

  m->current_offset_cal_pending = true;
  m->current_offset_calibrating = false;
  m->current_offset_cal_done = false;

  while (!m->current_offset_cal_done)
  {
    board_get_phase_current(m);
    board_current_offset_cal_step(m);
  }
}

void board_apply_phase_current(motor_handle_t *m)
{
  uint32_t i;
  const float kv = BOARD_ADC_VOLTS_PER_LSB;
  const float gain = BOARD_SO_AMPS_PER_VOLT;
  motor_state_t *state;

  if (m == NULL)
  {
    return;
  }

  state = &m->state;

  /*
   * 未完成零漂或未信任 offset 期间：必须用标称 V_BIAS 路径；adc_offset[] 在未写完前常为 0，
   * 不能当作零点。
   */
  for (i = 0; i < 3; i++)
  {
    float v_pin;

    if (!m->current_offset_cal_done)
    {
      v_pin = (float)state->phase_current.adc_raw[i] * kv;
      state->phase_current.ampere[i] = (v_pin - BOARD_SO_BIAS_V) * gain;
    }
    else
    {
      v_pin = (float)((int32_t)state->phase_current.adc_raw[i] -
                      (int32_t)state->phase_current.adc_offset[i]) *
              kv;
      state->phase_current.ampere[i] = v_pin * gain;
    }
  }

  state->board_temp.value =
      (float)state->board_temp.adc_raw * BOARD_ADC_VREF_V / BOARD_ADC_FULL_SCALE;
}

/** 须在 board_get_phase_current() 更新 raw 并完成 board_apply_phase_current() 之后调用 */
void board_current_offset_cal_step(motor_handle_t *m)
{
  uint32_t i;
  static uint32_t s_cnt;
  static uint32_t s_sum[3];
  static uint8_t s_active;

  if (m == NULL)
  {
    return;
  }

  if (m->current_offset_cal_done)
  {
    s_active = 0U;
    return;
  }

  if ((!m->current_offset_cal_pending) && (!s_active))
  {
    return;
  }

  if (!s_active)
  {
    s_active = 1U;
    s_cnt = 0U;
    s_sum[0] = 0U;
    s_sum[1] = 0U;
    s_sum[2] = 0U;
    m->current_offset_cal_pending = false;
    m->current_offset_calibrating = true;
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
    m->current_offset_calibrating = false;
    m->current_offset_cal_done = true;
    s_active = 0U;
  }
}
