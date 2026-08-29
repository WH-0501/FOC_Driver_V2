#include <stdio.h>
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
      /* 等待 mc_enable / foc_fsm_request_running 显式请求，不自动进入标定链 */
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

void foc_fsm_request_running(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
  }

  if (m->fsm != STATE_IDLE)
  {
    return;
  }

  if (motor_fault_is_latched(m) || !m->current_offset_cal_done)
  {
    return;
  }

  if (!control_mode_needs_pwm(m->ctrl_mode))
  {
    return;
  }

  /* 零漂/电角对齐已在 foc_init 完成；运行时 enable 直达 RUNNING。
   * 编码器/惯量等标定状态由未来独立 API 进入，不复用每次 enable。 */
  foc_fsm_next_state(m, STATE_RUNNING);
}

bool foc_fsm_is_calibrating(fsm_state_t state)
{
  return (state >= STATE_ENCODER_CALIBRATION) && (state <= STATE_ANTICOGGING);
}

void foc_fsm_request_calibration(motor_handle_t *m, fsm_state_t entry_state)
{
  if (m == NULL)
  {
    return;
  }

  if (m->fsm != STATE_IDLE)
  {
    return;
  }

  if (motor_fault_is_latched(m) || !m->current_offset_cal_done)
  {
    return;
  }

  if (!foc_fsm_is_calibrating(entry_state))
  {
    return;
  }

  foc_fsm_next_state(m, entry_state);
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
