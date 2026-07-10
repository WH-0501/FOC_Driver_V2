#include "mc_interface.h"
#include "foc.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "board.h"
#include "motor_driver.h"
#include "gate/gate_driver.h"

static int mc_bind_gate_driver(void)
{
  gate_driver_init_t cfg;

  cfg.type = GATE_DRIVER_TYPE_MP6540;
  if (board_get_gate_hw(&cfg.hw) != 0)
  {
    return MC_INIT_ERR_GATE;
  }

  if (gate_driver_init(&cfg) != GATE_DRIVER_INIT_OK)
  {
    return MC_INIT_ERR_GATE;
  }

  return MC_INIT_OK;
}

static int mc_platform_init(void)
{
  board_init();

  if (mc_bind_gate_driver() != MC_INIT_OK)
  {
    return MC_INIT_ERR_GATE;
  }

  motor_driver_init();

  return MC_INIT_OK;
}

static void mc_runtime_start(void)
{
  board_register_current_loop_callback(mc_current_loop_isr);
  board_current_loop_start();
}

int mc_init(const motor_config_t *config)
{
  if (mc_platform_init() != MC_INIT_OK)
  {
    return MC_INIT_ERR_GATE;
  }

  foc_init(config);
  mc_runtime_start();

  return MC_INIT_OK;
}

void mc_poll(void)
{
  uint8_t i;

  for (i = 0u; i < MOTOR_AXIS_COUNT; i++)
  {
    motor_fault_protect(&motor_axis_get(i)->handle);
  }
}

void mc_current_loop_isr(void)
{
  uint8_t i;

  for (i = 0u; i < MOTOR_AXIS_COUNT; i++)
  {
    foc_control_loop(&motor_axis_get(i)->handle);
  }
}

void mc_set_ctrl_mode(uint8_t axis_id, control_mode_t mode)
{
  motor_axis_t *axis = motor_axis_get(axis_id);

  if (axis == NULL)
  {
    return;
  }

  axis->handle.ctrl_mode = mode;

  if ((mode == CTRL_MODE_IDLE) && (axis->handle.fsm == STATE_RUNNING))
  {
    foc_fsm_next_state(&axis->handle, STATE_IDLE);
  }
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
  motor_axis_t *axis = motor_axis_get(axis_id);
  motor_handle_t *m;

  if (axis == NULL)
  {
    return 0;
  }

  m = &axis->handle;
  if (!motor_fault_is_latched(m))
  {
    return 0;
  }

  motor_fault_clear_latched(m, (fault_t)0xFFFFFFFFu);
  motor_fault_clear_status(m, (fault_t)0xFFFFFFFFu);
  foc_fsm_next_state(m, STATE_IDLE);
  return 1;
}
