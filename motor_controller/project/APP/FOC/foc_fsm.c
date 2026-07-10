#include "foc_fsm.h"
#include "control_mode.h"
#include "motor_axis.h"
#include "foc_fault.h"

#ifndef FOC_DEFAULT_CTRL_MODE
#define FOC_DEFAULT_CTRL_MODE CTRL_MODE_SPEED_OPEN_LOOP
#endif

static void foc_fsm_on_enter(motor_axis_t *axis, fsm_state_t state)
{
  motor_handle_t *m;

  if (axis == NULL)
  {
    return;
  }

  m = &axis->handle;

  switch (state)
  {
    case STATE_IDLE:
      motor_axis_disarm(m);
      break;
    case STATE_ENCODER_CALIBRATION:
    case STATE_RSLS_CALIBRATION:
    case STATE_FLUX_CALIBRATION:
    case STATE_ANTICOGGING:
      /* TODO: 运行时标定模块；占位直接跳过 */
      foc_fsm_next_state(m, (fsm_state_t)(state + 1));
      break;
    case STATE_ELECTRICAL_ALIGNMENT:
      if (axis->electrical_align_done || !m->config.auto_align_electrical)
      {
        foc_fsm_next_state(m, STATE_ANTICOGGING);
      }
      break;
    case STATE_RUNNING:
      break;
    case STATE_FAULT:
      break;
    default:
      break;
  }
}

static void foc_fsm_on_run(motor_axis_t *axis, fsm_state_t state)
{
  motor_handle_t *m;

  if (axis == NULL)
  {
    return;
  }

  m = &axis->handle;

  switch (state)
  {
    case STATE_IDLE:
      if (control_mode_needs_pwm(m->ctrl_mode) &&
          !motor_fault_is_latched(m) &&
          m->current_offset_cal_done)
      {
        foc_fsm_next_state(m, STATE_ENCODER_CALIBRATION);
      }
      break;
    case STATE_ENCODER_CALIBRATION:
    case STATE_RSLS_CALIBRATION:
    case STATE_FLUX_CALIBRATION:
    case STATE_ELECTRICAL_ALIGNMENT:
    case STATE_ANTICOGGING:
      break;
    case STATE_RUNNING:
      if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
      {
        break;
      }
      motor_axis_sync_power(m);
      if (m->ctrl_mode == CTRL_MODE_IDLE)
      {
        foc_fsm_next_state(m, STATE_IDLE);
      }
      break;
    case STATE_FAULT:
      break;
    default:
      break;
  }
}

void foc_fsm_init(motor_handle_t *m)
{
  motor_axis_t *axis;

  if (m == NULL)
  {
    return;
  }

  axis = motor_axis_from_handle(m);
  if (axis == NULL)
  {
    return;
  }

  axis->fsm_enter_pending = 0u;
  m->ctrl_mode = FOC_DEFAULT_CTRL_MODE;
  m->fsm = STATE_IDLE;
}

void foc_fsm_next_state(motor_handle_t *m, fsm_state_t next_state)
{
  motor_axis_t *axis;

  if ((m == NULL) || (next_state >= STATE_MAX))
  {
    return;
  }

  if (m->fsm == next_state)
  {
    return;
  }

  m->fsm = next_state;
  axis = motor_axis_from_handle(m);
  if (axis != NULL)
  {
    axis->fsm_enter_pending = 1u;
  }
}

fsm_state_t foc_fsm_get_state(const motor_handle_t *m)
{
  if (m == NULL)
  {
    return STATE_IDLE;
  }

  return m->fsm;
}

void foc_on_fault_latched(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
  }

  if (m->fsm == STATE_FAULT)
  {
    return;
  }

  motor_axis_shutdown(m);
  foc_fsm_next_state(m, STATE_FAULT);
}

void foc_fsm_loop(motor_handle_t *m)
{
  motor_axis_t *axis;

  if (m == NULL)
  {
    return;
  }

  axis = motor_axis_from_handle(m);
  if (axis == NULL)
  {
    return;
  }

  while (axis->fsm_enter_pending)
  {
    axis->fsm_enter_pending = 0u;
    foc_fsm_on_enter(axis, m->fsm);
  }

  foc_fsm_on_run(axis, m->fsm);
}
