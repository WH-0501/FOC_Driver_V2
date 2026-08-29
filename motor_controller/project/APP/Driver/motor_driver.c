#include "motor_driver.h"
#include "board.h"

static error_t motor_driver_bind_gate(motor_driver_t *drv, uint8_t axis_id)
{
  gate_driver_init_t cfg;

  if (drv == NULL)
  {
    return ERR_FAIL;
  }

  cfg.type = GATE_DRIVER_TYPE_MP6540;
  if (board_get_gate_hw(axis_id, &cfg.hw) != 0)
  {
    drv->gate = NULL;
    return ERR_FAIL;
  }

  MP6540_Init(&drv->mp6540, &cfg.hw);
  drv->gate = MP6540_AsGateDriver(&drv->mp6540);
  if (drv->gate == NULL)
  {
    return ERR_FAIL;
  }

  return ERR_NONE;
}

error_t motor_driver_init(motor_driver_t *drv, uint8_t axis_id)
{
  if (drv == NULL)
  {
    return ERR_FAIL;
  }

  drv->armed = false;
  drv->axis_id = axis_id;
  drv->gate = NULL;

  if (motor_driver_bind_gate(drv, axis_id) != ERR_NONE)
  {
    return ERR_FAIL;
  }

  motor_driver_shutdown(drv);
  return ERR_NONE;
}

const gate_driver_t *motor_driver_get_gate(const motor_driver_t *drv)
{
  if (drv == NULL)
  {
    return NULL;
  }

  return drv->gate;
}

void motor_driver_arm(motor_driver_t *drv)
{
  if ((drv == NULL) || (drv->gate == NULL) || drv->armed)
  {
    return;
  }

  (void)pwm_hw_init(drv->axis_id);
  gate_driver_exit_sleep(drv->gate);
  gate_driver_set_all_phase_en(drv->gate, true);
  (void)pwm_hw_start(drv->axis_id);
  drv->armed = true;
}

void motor_driver_disarm(motor_driver_t *drv)
{
  if ((drv == NULL) || (drv->gate == NULL))
  {
    return;
  }

  (void)pwm_hw_stop(drv->axis_id);
  gate_driver_exit_sleep(drv->gate);
  gate_driver_set_all_phase_en(drv->gate, false);
  drv->armed = false;
}

void motor_driver_shutdown(motor_driver_t *drv)
{
  if ((drv == NULL) || (drv->gate == NULL))
  {
    return;
  }

  (void)pwm_hw_stop(drv->axis_id);
  gate_driver_enter_inactive_state(drv->gate);
  drv->armed = false;
}

bool motor_driver_is_armed(const motor_driver_t *drv)
{
  if (drv == NULL)
  {
    return false;
  }

  return drv->armed;
}

void motor_driver_prepare_dc_offset_cal(motor_driver_t *drv)
{
  if ((drv == NULL) || (drv->gate == NULL))
  {
    return;
  }

  (void)pwm_hw_init(drv->axis_id);
  gate_driver_exit_sleep(drv->gate);
  gate_driver_set_all_phase_en(drv->gate, true);
  (void)pwm_hw_lowside_brake_on(drv->axis_id);
  (void)pwm_hw_start(drv->axis_id);
  drv->armed = false;
}

error_t motor_driver_set_pwm(motor_driver_t *drv, const motor_actuation_t *actuation)
{
  if ((drv == NULL) || !drv->armed)
  {
    return ERR_FAIL;
  }

  if (actuation == NULL)
  {
    return ERR_FAIL;
  }

  return set_pwm(drv->axis_id, (motor_actuation_t *)actuation);
}
