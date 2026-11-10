#include "foc_sense.h"
#include "encoder.h"
#include "current_sense.h"
#include "foc_filter.h"
#include "foc_math.h"
#include "board.h"
#include "common.h"
#if FOC_PARK_USE_SINCOS_LUT
#include "foc_sense_trig.h"
#endif
#include "dwt_profile_delay.h"
#include "motor.h"

void foc_sense_poll_vbus(motor_handle_t *m)
{
  static uint32_t s_last_us;
  static uint8_t s_vbus_lpf_seeded;

  uint32_t now;

  if (m == NULL)
  {
    return;
  }

  now = (uint32_t)dwt_get_ticks_us();
  if ((now - s_last_us) < FOC_VBUS_POLL_MIN_INTERVAL_US)
  {
    return;
  }
  s_last_us = now;

  board_get_vbus_and_temp(m);
  if (s_vbus_lpf_seeded == 0u)
  {
    lpf1_reset(&m->vbus_lpf, m->state.vbus);
    s_vbus_lpf_seeded = 1u;
  }
  m->state.vbus_filtered = lpf1_update(&m->vbus_lpf, m->state.vbus);
}

void foc_sense_update(motor_handle_t *m)
{
  /* Alignment/calibration consumes the shaft angle, not the outer-loop feedback. */
  foc_sense_update_timed(m, NULL, false);
}

/**
 * @brief 传感器更新
 * 
 * @param m 电机句柄
 * @param marks 传感器标记
 * @param outer_tick 外环跳过标志
 */
void foc_sense_update_timed(motor_handle_t *m, foc_sense_marks_t *marks, bool outer_tick)
{
  float id = 0.0f;
  float iq = 0.0f;

  if (m == NULL)
  {
    return;
  }

  motor_runtime_t *rt = motor_runtime(m);
  bool measure_sense = rt != NULL && !rt->profile_cst_outer;
  uint32_t sense_start_ticks = measure_sense ? dwt_get_ticks() : 0u;
  board_get_phase_current(m);
  current_sense_process_sample(m);
  uint32_t current_end_ticks = (measure_sense || marks != NULL) ? dwt_get_ticks() : 0u;

  if (marks != NULL) marks->current_end = current_end_ticks;

  encoder_update(m, outer_tick);
  if (marks != NULL) marks->position_end = dwt_get_ticks();
  if (outer_tick) (void)encoder_estimate_velocity(m);
  uint32_t encoder_end_ticks = (measure_sense || marks != NULL) ? dwt_get_ticks() : 0u;
  if (marks != NULL) marks->encoder_end = encoder_end_ticks;

  clarke_transform(m->state.phase_current.ampere[0],
                   m->state.phase_current.ampere[1],
                   m->state.phase_current.ampere[2],
                   &m->state.i_alpha, &m->state.i_beta);

  {
    float s;
    float c;

#if FOC_PARK_USE_SINCOS_LUT
    foc_sense_sincos(m->state.theta_elec_rad, &s, &c);
#else
    foc_sincos(m->state.theta_elec_rad, &s, &c);
#endif
#if FOC_REUSE_PARK_SINCOS
    m->state.sin_elec = s;
    m->state.cos_elec = c;
#endif
    park_transform_sc(m->state.i_alpha, m->state.i_beta, s, c, &id, &iq);
  }

  m->state.i_d = id;
  m->state.i_q = iq;

  m->state.i_bus = sqrtf(SQ(id) + SQ(iq));
  m->state.i_mod = lpf1_update(&m->i_mod_lpf, m->state.i_bus * 1000.0f);

  if (measure_sense)
  {
    uint32_t sense_end_ticks = dwt_get_ticks();
    rt->foc_sense_update_us = dwt_elapsed_us(sense_start_ticks, sense_end_ticks);
    rt->foc_current_sense_us = dwt_elapsed_us(sense_start_ticks, current_end_ticks);
    rt->foc_encoder_update_us = dwt_elapsed_us(current_end_ticks, encoder_end_ticks);
  }
}
