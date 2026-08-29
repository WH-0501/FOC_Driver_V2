#include "foc_sense.h"
#include "motor_axis.h"
#include "current_sense.h"
#include "foc_filter.h"
#include "foc_math.h"
#include "board.h"

void foc_sense_update(motor_handle_t *m)
{
  motor_axis_t *axis;
  float id = 0.0f;
  float iq = 0.0f;
  float i_mod;

  if (m == NULL)
  {
    return;
  }

  axis = motor_axis_from_handle(m);
  if ((axis == NULL) || !motor_axis_is_present(axis))
  {
    return;
  }

  motor_axis_update_encoders(axis);
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
