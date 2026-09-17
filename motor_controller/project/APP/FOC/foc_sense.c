#include "foc_sense.h"
#include "encoder.h"
#include "current_sense.h"
#include "foc_filter.h"
#include "foc_math.h"
#include "board.h"
#include "common.h"

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
  float id = 0.0f;
  float iq = 0.0f;
  float i_mod;

  if (m == NULL)
  {
    return;
  }

  board_get_phase_current(m);
  current_sense_process_sample(m);
  encoder_update(m);

  clarke_transform(m->state.phase_current.ampere[0],
                   m->state.phase_current.ampere[1],
                   m->state.phase_current.ampere[2],
                   &m->state.i_alpha, &m->state.i_beta);

  park_transform(m->state.i_alpha, m->state.i_beta,
                 m->state.theta_elec_rad,
                 &id, &iq);

  /* 电流环 PI 用未滤波瞬时值；LPF 仅用于 i_mod 观测/显示 */
  m->state.i_d = id;
  m->state.i_q = iq;

  i_mod = sqrtf(SQ(id) + SQ(iq)) * 1000.0f;
  m->state.i_mod = lpf1_update(&m->i_mod_lpf, i_mod);
}
