#include "mc_interface.h"
#include "foc.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "board.h"
#include "motor.h"
#include "control_mode.h"
#include "error_types.h"

#ifndef MC_DEFAULT_ENABLE_MODE
#define MC_DEFAULT_ENABLE_MODE CTRL_MODE_SPEED_OPEN_LOOP
#endif

static mc_error_t mc_platform_init(void)
{
  board_init();
  return MC_OK;
}

static mc_error_t mc_runtime_start(void)
{
  board_register_current_loop_callback(mc_current_loop_isr);
  if (current_hw_init() != ERR_NONE)
  {
    return MC_INIT_ERR_CURRENT_LOOP;
  }
  return MC_OK;
}

mc_error_t mc_init(const motor_config_t *config)
{
  mc_error_t ret;
  motor_handle_t *m;

  if (config == NULL)
  {
    return MC_INIT_ERR_CONFIG;
  }

  ret = mc_platform_init();
  if (ret != MC_OK)
  {
    return ret;
  }

  if (motor_hw_init() != ERR_NONE)
  {
    return MC_INIT_ERR_GATE_HW;
  }

  m = motor_get_handler();
  foc_init(m, config);

  ret = mc_runtime_start();
  if (ret != MC_OK)
  {
    return ret;
  }

  return MC_OK;
}

void mc_poll(void)
{
  motor_fault_protect(motor_get_handler());
}

void mc_current_loop_isr(void)
{
  foc_control_loop(motor_get_handler());
}

void mc_set_ctrl_mode(control_mode_t mode)
{
  motor_handle_t *m = motor_get_handler();

  m->ctrl_mode = mode;

  if (mode == CTRL_MODE_IDLE)
  {
    mc_disable();
  }
}

mc_error_t mc_enable(void)
{
  motor_handle_t *m = motor_get_handler();

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return MC_ERR_FAULT;
  }

  if (!m->current_offset_cal_done)
  {
    return MC_ERR_OFFSET_CAL;
  }

  if (m->ctrl_mode == CTRL_MODE_IDLE)
  {
    m->ctrl_mode = MC_DEFAULT_ENABLE_MODE;
  }

  if (!control_mode_needs_pwm(m->ctrl_mode))
  {
    return MC_ERR_CTRL_MODE;
  }

  if (m->fsm == STATE_RUNNING)
  {
    return MC_OK;
  }

  if (foc_fsm_is_calibrating(m->fsm))
  {
    return MC_ERR_CALIBRATING;
  }

  if (m->fsm != STATE_IDLE)
  {
    return MC_ERR_STATE;
  }

  foc_fsm_request_running(m);
  return (m->fsm == STATE_RUNNING) ? MC_OK : MC_ERR_STATE;
}

static fsm_state_t mc_cal_type_to_fsm_entry(mc_calibration_type_t type)
{
  switch (type)
  {
    case MC_CAL_ENCODER:
      return STATE_ENCODER_CALIBRATION;
    case MC_CAL_RSLS:
      return STATE_RSLS_CALIBRATION;
    case MC_CAL_FLUX:
      return STATE_FLUX_CALIBRATION;
    case MC_CAL_ELECTRICAL_ALIGN:
      return STATE_ELECTRICAL_ALIGNMENT;
    case MC_CAL_ANTICOGGING:
      return STATE_ANTICOGGING;
    case MC_CAL_SEQUENCE:
      return STATE_ENCODER_CALIBRATION;
    default:
      return STATE_MAX;
  }
}

static mc_calibration_type_t mc_fsm_state_to_cal_type(fsm_state_t state)
{
  switch (state)
  {
    case STATE_ENCODER_CALIBRATION:
      return MC_CAL_ENCODER;
    case STATE_RSLS_CALIBRATION:
      return MC_CAL_RSLS;
    case STATE_FLUX_CALIBRATION:
      return MC_CAL_FLUX;
    case STATE_ELECTRICAL_ALIGNMENT:
      return MC_CAL_ELECTRICAL_ALIGN;
    case STATE_ANTICOGGING:
      return MC_CAL_ANTICOGGING;
    default:
      return MC_CAL_NONE;
  }
}

int mc_start_calibration(mc_calibration_type_t type)
{
  motor_handle_t *m = motor_get_handler();
  fsm_state_t entry;

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return MC_ERR_FAULT;
  }

  if (m->fsm != STATE_IDLE)
  {
    return MC_ERR_STATE;
  }

  entry = mc_cal_type_to_fsm_entry(type);
  if (entry >= STATE_MAX)
  {
    return MC_ERR_CAL_TYPE;
  }

  foc_fsm_request_calibration(m, entry);
  return foc_fsm_is_calibrating(m->fsm) ? MC_OK : MC_ERR_STATE;
}

void mc_stop_calibration(void)
{
  motor_handle_t *m = motor_get_handler();

  if (!foc_fsm_is_calibrating(m->fsm))
  {
    return;
  }

  if (m->fsm != STATE_FAULT)
  {
    foc_fsm_next_state(m, STATE_IDLE);
  }
}

bool mc_is_calibrating(void)
{
  return foc_fsm_is_calibrating(motor_get_handler()->fsm);
}

mc_calibration_type_t mc_get_calibration_type(void)
{
  return mc_fsm_state_to_cal_type(motor_get_handler()->fsm);
}

mc_error_t mc_disable(void)
{
  motor_handle_t *m = motor_get_handler();

  m->ctrl_mode = CTRL_MODE_IDLE;

  if (foc_fsm_is_calibrating(m->fsm))
  {
    mc_stop_calibration();
    return MC_ERR_CALIBRATING;
  }

  if (m->fsm != STATE_FAULT)
  {
    foc_fsm_next_state(m, STATE_IDLE);
  }
  return MC_OK;
}

bool mc_is_enabled(void)
{
  motor_handle_t *m = motor_get_handler();

  return (m->fsm == STATE_RUNNING) && motor_is_armed(m);
}

control_mode_t mc_get_ctrl_mode(void)
{
  return motor_get_handler()->ctrl_mode;
}

fsm_state_t mc_get_fsm_state(void)
{
  return foc_fsm_get_state(motor_get_handler());
}

int mc_try_recover_from_fault(void)
{
  motor_handle_t *m = motor_get_handler();

  if (!motor_fault_is_latched(m))
  {
    return MC_ERR_NO_FAULT;
  }

  motor_fault_clear_latched(m, (fault_t)0xFFFFFFFFu);
  motor_fault_clear_status(m, (fault_t)0xFFFFFFFFu);
  foc_fsm_next_state(m, STATE_IDLE);
  return MC_OK;
}

int mc_set_joint_origin(void)
{
  motor_handle_t *m = motor_get_handler();

  if (mc_is_enabled())
  {
    return MC_ERR_STATE;
  }

  m->config.param.theta_offset_rad += m->state.shaft.position_rad;
  return MC_OK;
}

int mc_align_electrical(void)
{
  motor_handle_t *m = motor_get_handler();

  if (mc_is_enabled())
  {
    (void)mc_disable();
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return MC_ERR_FAULT;
  }

  if (m->fsm != STATE_IDLE)
  {
    return MC_ERR_STATE;
  }

  foc_align_electrical(m);
  return MC_OK;
}
