#include "mc_interface.h"
#include "foc.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "board.h"
#include "motor_axis.h"
#include "control_mode.h"
#include "error_types.h"
#include "motor_config.h"

#ifndef MC_DEFAULT_ENABLE_MODE
#define MC_DEFAULT_ENABLE_MODE CTRL_MODE_SPEED_OPEN_LOOP
#endif

static motor_axis_t *mc_axis_get_active(uint8_t axis_id)
{
  motor_axis_t *axis = motor_axis_get(axis_id);

  if ((axis == NULL) || !motor_axis_is_present(axis))
  {
    return NULL;
  }

  return axis;
}

/**
 * @brief 平台初始化（全局一次：板级，不绑定具体轴）
 */
static mc_error_t mc_platform_init(void)
{
  board_init();
  return MC_OK;
}

/**
 * @brief 单轴 bring-up：硬件（driver + 编码器）+ 该轴 FOC 控制栈
 * @param axis_id: 轴号
 * @param config: 系统配置
 * @return MC_OK: 成功
 */
static mc_error_t mc_init_axis(uint8_t axis_id, const motor_system_config_t *config)
{
  motor_axis_t *axis = motor_axis_get(axis_id);

  if ((axis == NULL) || (config == NULL))
  {
    return MC_INIT_ERR_AXIS;
  }

  if (motor_axis_is_present(axis))
  {
    if (motor_axis_init_hw(axis) != ERR_NONE)
    {
      return MC_INIT_ERR_GATE_HW;
    }
  }
  else
  {
    (void)motor_axis_init_hw(axis);
  }

  foc_init_axis(axis, &config->axis[axis_id]);
  return MC_OK;
}

/**
 * @brief 运行时启动：注册电流环回调 + 初始化电流环硬件
 * 
 * @return MC_OK: 成功
 * @return MC_INIT_ERR_CURRENT_LOOP: 电流环硬件初始化失败
 */
static mc_error_t mc_runtime_start(void)
{
  board_register_current_loop_callback(mc_current_loop_isr);
  if (current_hw_init() != ERR_NONE)
  {
    return MC_INIT_ERR_CURRENT_LOOP;
  }
  return MC_OK;
}

mc_error_t mc_init(const motor_system_config_t *config)
{
  mc_error_t ret;
  uint8_t i;

  if (config == NULL)
  {
    return MC_INIT_ERR_CONFIG;
  }

  ret = mc_platform_init();
  if (ret != MC_OK)
  {
    return ret;
  }

  for (i = 0u; i < MOTOR_AXIS_COUNT; i++)
  {
    ret = mc_init_axis(i, config);
    if (ret != MC_OK)
    {
      return ret;
    }
  }

  ret = mc_runtime_start();
  if (ret != MC_OK)
  {
    return ret;
  }

  return MC_OK;
}

void mc_poll(void)
{
  uint8_t i;

  for (i = 0u; i < MOTOR_AXIS_COUNT; i++)
  {
    if (motor_axis_is_present_id(i))
    {
      motor_fault_protect(&motor_axis_get(i)->handle);
    }
  }
}

void mc_current_loop_isr(void)
{
  uint8_t i;

  for (i = 0u; i < MOTOR_AXIS_COUNT; i++)
  {
    if (motor_axis_is_present_id(i))
    {
      foc_control_loop(&motor_axis_get(i)->handle);
    }
  }
}

void mc_set_ctrl_mode(uint8_t axis_id, control_mode_t mode)
{
  motor_axis_t *axis = mc_axis_get_active(axis_id);

  if (axis == NULL)
  {
    return;
  }

  axis->handle.ctrl_mode = mode;

  if (mode == CTRL_MODE_IDLE)
  {
    mc_disable(axis_id);
  }
}

mc_error_t mc_enable(uint8_t axis_id)
{
  motor_axis_t *axis = mc_axis_get_active(axis_id);
  motor_handle_t *m;

  if (axis == NULL)
  {
    if (motor_axis_get(axis_id) == NULL)
    {
      return MC_ERR_AXIS_INVALID;
    }
    return MC_ERR_AXIS_INACTIVE;
  }

  m = &axis->handle;
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

int mc_start_calibration(uint8_t axis_id, mc_calibration_type_t type)
{
  motor_axis_t *axis = mc_axis_get_active(axis_id);
  motor_handle_t *m;
  fsm_state_t entry;

  if (axis == NULL)
  {
    return (motor_axis_get(axis_id) == NULL) ? MC_ERR_AXIS_INVALID : MC_ERR_AXIS_INACTIVE;
  }

  m = &axis->handle;
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

void mc_stop_calibration(uint8_t axis_id)
{
  motor_axis_t *axis = mc_axis_get_active(axis_id);
  motor_handle_t *m;

  if (axis == NULL)
  {
    return;
  }

  m = &axis->handle;
  if (!foc_fsm_is_calibrating(m->fsm))
  {
    return;
  }

  if (m->fsm != STATE_FAULT)
  {
    foc_fsm_next_state(m, STATE_IDLE);
  }
}

bool mc_is_calibrating(uint8_t axis_id)
{
  motor_axis_t *axis = motor_axis_get(axis_id);

  if (axis == NULL)
  {
    return false;
  }

  return foc_fsm_is_calibrating(axis->handle.fsm);
}

mc_calibration_type_t mc_get_calibration_type(uint8_t axis_id)
{
  motor_axis_t *axis = motor_axis_get(axis_id);

  if (axis == NULL)
  {
    return MC_CAL_NONE;
  }

  return mc_fsm_state_to_cal_type(axis->handle.fsm);
}

mc_error_t mc_disable(uint8_t axis_id)
{
  motor_axis_t *axis = mc_axis_get_active(axis_id);
  motor_handle_t *m;

  if (axis == NULL)
  {
    if (motor_axis_get(axis_id) == NULL)
    {
      return MC_ERR_AXIS_INVALID;
    }
    return MC_OK;
  }

  m = &axis->handle;
  m->ctrl_mode = CTRL_MODE_IDLE;

  if (foc_fsm_is_calibrating(m->fsm))
  {
    mc_stop_calibration(axis_id);
    return MC_ERR_CALIBRATING;
  }

  if (m->fsm != STATE_FAULT)
  {
    foc_fsm_next_state(m, STATE_IDLE);
  }
  return MC_OK;
}

bool mc_is_enabled(uint8_t axis_id)
{
  motor_axis_t *axis = motor_axis_get(axis_id);

  if (axis == NULL)
  {
    return false;
  }

  return (axis->handle.fsm == STATE_RUNNING) && motor_axis_is_armed(&axis->handle);
}

control_mode_t mc_get_ctrl_mode(uint8_t axis_id)
{
  motor_axis_t *axis = motor_axis_get(axis_id);

  if (axis == NULL)
  {
    return CTRL_MODE_IDLE;
  }

  return axis->handle.ctrl_mode;
}

fsm_state_t mc_get_fsm_state(uint8_t axis_id)
{
  motor_axis_t *axis = motor_axis_get(axis_id);

  if (axis == NULL)
  {
    return STATE_IDLE;
  }

  return foc_fsm_get_state(&axis->handle);
}

int mc_try_recover_from_fault(uint8_t axis_id)
{
  motor_axis_t *axis = mc_axis_get_active(axis_id);
  motor_handle_t *m;

  if (axis == NULL)
  {
    return (motor_axis_get(axis_id) == NULL) ? MC_ERR_AXIS_INVALID : MC_ERR_AXIS_INACTIVE;
  }

  m = &axis->handle;
  if (!motor_fault_is_latched(m))
  {
    return MC_ERR_NO_FAULT;
  }

  motor_fault_clear_latched(m, (fault_t)0xFFFFFFFFu);
  motor_fault_clear_status(m, (fault_t)0xFFFFFFFFu);
  foc_fsm_next_state(m, STATE_IDLE);
  return MC_OK;
}
