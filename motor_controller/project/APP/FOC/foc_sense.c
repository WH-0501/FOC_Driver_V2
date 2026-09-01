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

  now = board_time_us();
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

  encoder_update(m);
  board_get_phase_current(m);
  current_sense_process_sample(m);

  clarke_transform(m->state.phase_current.ampere[0],
                   m->state.phase_current.ampere[1],
                   m->state.phase_current.ampere[2],
                   &m->state.i_alpha, &m->state.i_beta);

  park_transform(m->state.i_alpha, m->state.i_beta,
                 m->state.theta_elec_rad,
                 &id, &iq);

  m->state.i_d = lpf1_update(&m->i_d_lpf, id);
  m->state.i_q = lpf1_update(&m->i_q_lpf, iq);

  i_mod = sqrtf(SQ(m->state.i_d) + SQ(m->state.i_q)) * 1000.0f;
  m->state.i_mod = lpf1_update(&m->i_mod_lpf, i_mod);
}
