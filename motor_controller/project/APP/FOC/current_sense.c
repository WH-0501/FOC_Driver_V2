#include "current_sense.h"
#include "board.h"
#include "motor.h"
#include "foc_fault.h"

static void current_sense_offset_cal_step(motor_handle_t *m)
{
  motor_runtime_t *rt;
  uint32_t i;

  if (m == NULL)
  {
    return;
  }

  rt = motor_runtime(m);
  if (rt == NULL)
  {
    return;
  }

  if (m->current_offset_cal_done)
  {
    rt->offset_active = 0u;
    return;
  }

  if ((m->current_offset_cal_pending == 0u) && (rt->offset_active == 0u))
  {
    return;
  }

  if (rt->offset_active == 0u)
  {
    rt->offset_active = 1u;
    rt->offset_cnt = 0U;
    rt->offset_sum[0] = 0U;
    rt->offset_sum[1] = 0U;
    rt->offset_sum[2] = 0U;
    m->current_offset_cal_pending = false;
    m->current_offset_calibrating = true;
  }

  for (i = 0U; i < 3U; i++)
  {
    rt->offset_sum[i] += (uint32_t)m->state.phase_current.adc_raw[i];
  }
  rt->offset_cnt++;

  if (rt->offset_cnt >= CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT)
  {
    for (i = 0U; i < 3U; i++)
    {
      m->state.phase_current.adc_offset[i] =
          (uint16_t)(rt->offset_sum[i] >> CURRENT_OFFSET_CALIBRATION_TIME);
    }
    m->current_offset_calibrating = false;
    m->current_offset_cal_done = true;
    rt->offset_active = 0u;
  }
}

static void current_sense_apply_raw_to_physical(motor_handle_t *m)
{
  uint32_t i;
  const float kv = CURRENT_SENSE_ADC_VOLTS_PER_LSB;
  const float gain = CURRENT_SENSE_SO_AMPS_PER_VOLT * CURRENT_SENSE_PHASE_POLARITY;
  motor_state_t *state;

  if (m == NULL)
  {
    return;
  }

  state = &m->state;

  for (i = 0U; i < 3U; i++)
  {
    float v_pin = 0.0f;
    if (!m->current_offset_cal_done)
    {
      v_pin = (float)state->phase_current.adc_raw[i] * kv;
      state->phase_current.ampere[i] = (v_pin - CURRENT_SENSE_SO_BIAS_V) * gain;
    }
    else
    {
      v_pin = (float)((uint16_t)state->phase_current.adc_raw[i] - (uint16_t)state->phase_current.adc_offset[i]) * kv;
      state->phase_current.ampere[i] = v_pin * gain;
    }
  }

  /**
   * 三电阻采样时,基于基尔霍夫定律.使用均值滤波,抑制随机噪声和尖峰脉冲.current_avg
   * 
   * 原理: 假设测量误差服从正态分布,那么三相电流的均值就可被视为该时刻的共模误差,
   *       通过减去均值,能够在满足KCL约束的前提下,可以消除共模误差,提高电流测量的精度.
   */
  state->phase_current.kcl_residual = state->phase_current.ampere[0]
                                  + state->phase_current.ampere[1]
                                  + state->phase_current.ampere[2];
  float current_avg = state->phase_current.kcl_residual / 3.0f;
  state->phase_current.ampere[0] -= current_avg;
  state->phase_current.ampere[1] -= current_avg;
  state->phase_current.ampere[2] -= current_avg;

  state->motor_temp.value =
      (float)state->motor_temp.adc_raw * CURRENT_SENSE_ADC_VREF_V / CURRENT_SENSE_ADC_FULL_SCALE;
}

static void current_sense_run_offset_cal_blocking(motor_handle_t *m)
{
  motor_runtime_t *rt;
  uint32_t saved_state = 0U;
  uint32_t discard;
  error_t ret;
  if (m == NULL) { return; }
  rt = motor_runtime(m);
  if (rt == NULL) { return; }

  /* Take ownership before changing the bridge. Never run the control callback
   * concurrently with blocking calibration, and never stop ADC triggering. 
   * AT32 上 begin 会:
   *   - 禁止从 ISR 调用 (ERR_STATE)
   *   - 保存并关闭 ADC2 PCCE 完成中断（避免 ISR 抢清标志）
   *   - 除陈旧完成标志
   *   - 不停止 ADC 转换 / PWM 触发链
   */
  ret = current_hw_calibration_begin(&saved_state);
  if (ret != ERR_NONE)
  {
    /* 如果当前ADC正在使用,则直接返回 */
    m->current_offset_cal_pending = false;
    m->current_offset_calibrating = false;
    rt->offset_active = 0U;
    motor_disarm(m);
    motor_fault_raise(m, (fault_t)(FAULT_CURRENT_SENSOR | FAULT_CALIBRATION));
    return;
  }
  rt->offset_active = 0U;
  rt->offset_cnt = 0U;
  motor_enter_dc_cal_hw(m);
  /* Discard fresh conversions after wake/bridge preparation. At 20 kHz,
   * 128 events provide at least about 6.4 ms before accumulation starts. 
   * 稳定期: 丢弃 128 次新转换，唤醒、桥臂切换后的过渡样本不进均值计算
   */
  for (discard = 0U; discard < CURRENT_OFFSET_SETTLING_SAMPLES; ++discard)
  {
    ret = current_hw_calibration_read(m);
    if (ret != ERR_NONE) { break; }
  }
  while ((ret == ERR_NONE) && !m->current_offset_cal_done)
  {
    ret = current_hw_calibration_read(m);
    if (ret == ERR_NONE) { current_sense_offset_cal_step(m); }
  }

  /* Disable the bridge BEFORE restoring the normal ADC interrupt owner. */
  motor_disarm(m);
  rt->offset_active = 0U;
  m->current_offset_cal_pending = false;
  m->current_offset_calibrating = false;
  if (ret != ERR_NONE)
  {
    m->current_offset_cal_done = false;
    motor_fault_raise(m, (fault_t)(FAULT_CURRENT_SENSOR | FAULT_CALIBRATION));
  }
  current_hw_calibration_end(saved_state);
}

/**
 * @brief 运行电流采样偏移校准
 *        1. 设置电流采样偏移校准状态
 *        2. 清零电流采样偏移校准值
 * @param m 电机句柄
 */
void current_sense_init(motor_handle_t *m)
{
  uint32_t i;

  if (m == NULL)
  {
    return;
  }

  m->current_offset_cal_pending = true;
  m->current_offset_calibrating = false;
  m->current_offset_cal_done = false;

  for (i = 0U; i < 3U; i++)
  {
    m->state.phase_current.adc_offset[i] = 0U;
  }
  current_sense_run_offset_cal_blocking(m);
}

void current_sense_process_sample(motor_handle_t *m)
{
  current_sense_apply_raw_to_physical(m);
  /* Offset accumulation is owned exclusively by the fresh-event startup path. */
}
