#include "foc_sense.h"
#include "fra.h"
#include "encoder.h"
#include "current_sense.h"
#include "foc_filter.h"
#include "foc_math.h"
#include "board.h"
#include "common.h"
#include "dwt_profile_delay.h"
#include "motor.h"

static uint32_t foc_sense_elapsed_us(uint32_t t0_us)
{
  return (uint32_t)(dwt_get_ticks_us() - t0_us);
}

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
  uint32_t t0_us;
  uint32_t t_seg_us;
  motor_runtime_t *rt = motor_runtime(m);

  if (m == NULL)
  {
    return;
  }

  t0_us = dwt_get_ticks_us();
  board_get_phase_current(m);
  current_sense_process_sample(m);
  if (rt != NULL)
  {
    rt->foc_current_sense_us = foc_sense_elapsed_us(t0_us);
  }

  t_seg_us = dwt_get_ticks_us();
  if (fra_state() == FRA_STATE_ACQUIRING)
  {
    uint16_t encoder_t0_us = (uint16_t)board_time_us();

    encoder_update(m);
    fra_stage(FRA_STAGE_ENCODER,
              (uint16_t)((uint16_t)board_time_us() - encoder_t0_us));
  }
  else
  {
    encoder_update(m);
  }
  if (rt != NULL)
  {
    rt->foc_encoder_update_us = foc_sense_elapsed_us(t_seg_us);
  }

  clarke_transform(m->state.phase_current.ampere[0],
                   m->state.phase_current.ampere[1],
                   m->state.phase_current.ampere[2],
                   &m->state.i_alpha, &m->state.i_beta);

  park_transform(m->state.i_alpha, m->state.i_beta,
                 m->state.theta_elec_rad,
                 &id, &iq);

  m->state.i_d = id;
  m->state.i_q = iq;

  i_mod = sqrtf(SQ(id) + SQ(iq)) * 1000.0f;
  m->state.i_mod = lpf1_update(&m->i_mod_lpf, i_mod);
  if (rt != NULL)
  {
    rt->foc_sense_update_us = foc_sense_elapsed_us(t0_us);
  }
}
