#include "motor.h"
#include "control_mode.h"
#include "encoder.h"
#include "foc_fault.h"
#include "motor_driver.h"

static motor_handle_t s_motor_handle;
static motor_runtime_t s_motor_rt;
static motor_driver_t s_motor_driver;

motor_handle_t *motor_get_handler(void)
{
  return &s_motor_handle;
}

motor_runtime_t *motor_runtime(motor_handle_t *m)
{
  if (m == &s_motor_handle)
  {
    return &s_motor_rt;
  }

  return NULL;
}

const gate_driver_t *motor_gate(motor_handle_t *m)
{
  (void)m;
  return motor_driver_get_gate(&s_motor_driver);
}

error_t motor_hw_init(void)
{
  if (motor_driver_init(&s_motor_driver) != ERR_NONE)
  {
    return ERR_FAIL;
  }

  encoder_init();
  return ERR_NONE;
}

void motor_arm(motor_handle_t *m)
{
  if (motor_runtime(m) == NULL)
  {
    return;
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return;
  }

  motor_driver_arm(&s_motor_driver);
  m->out.pwm_enable = motor_driver_is_armed(&s_motor_driver) ? 1u : 0u;
}

void motor_disarm(motor_handle_t *m)
{
  if (motor_runtime(m) == NULL)
  {
    return;
  }

  m->out.pwm_enable = 0u;
  motor_driver_disarm(&s_motor_driver);
}

void motor_shutdown(motor_handle_t *m)
{
  if (motor_runtime(m) == NULL)
  {
    return;
  }

  m->out.pwm_enable = 0u;
  motor_driver_shutdown(&s_motor_driver);
}

bool motor_is_armed(const motor_handle_t *m)
{
  if (motor_runtime((motor_handle_t *)m) == NULL)
  {
    return false;
  }

  return motor_driver_is_armed(&s_motor_driver);
}

/**
 * @brief 同步电机驱动器使能：根据控制模式决定是否使能电机驱动器
 * 
 * @param m: 电机句柄
 * 
 * 1. 如果电机处于故障状态，则不进行同步
 * 2. 如果控制模式需要 PWM 输出，则使能电机驱动器
 * 3. 如果控制模式不需要 PWM 输出，则关闭电机驱动器
 * 
 * @return void
 */
void motor_sync_arm_with_mode(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return;
  }

  if (control_mode_needs_pwm(m->ctrl_mode))
  {
    if (!motor_is_armed(m))
    {
      motor_arm(m);
    }
  }
  else if (motor_is_armed(m))
  {
    motor_disarm(m);
  }
}

void motor_enter_dc_cal_hw(motor_handle_t *m)
{
  if (motor_runtime(m) == NULL)
  {
    return;
  }

  m->out.pwm_enable = 0u;
  motor_driver_prepare_dc_offset_cal(&s_motor_driver);
}

error_t motor_apply_pwm(motor_handle_t *m, const motor_actuation_t *actuation)
{
  if ((motor_runtime(m) == NULL) || (actuation == NULL))
  {
    return ERR_FAIL;
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return ERR_FAIL;
  }

  if (!motor_driver_is_armed(&s_motor_driver))
  {
    return ERR_FAIL;
  }

  return motor_driver_set_pwm(&s_motor_driver, actuation);
}
