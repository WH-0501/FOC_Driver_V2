#include "motor_axis.h"
#include "control_mode.h"
#include "motor_driver.h"
#include "foc_fault.h"

static motor_axis_t s_motor_axes[MOTOR_AXIS_COUNT] = {
  { .id = 0u },
};

motor_axis_t *motor_axis_get(uint8_t id)
{
  if (id >= MOTOR_AXIS_COUNT)
  {
    return NULL;
  }

  return &s_motor_axes[id];
}

motor_axis_t *motor_axis_from_handle(motor_handle_t *m)
{
  uint8_t i;

  if (m == NULL)
  {
    return NULL;
  }

  for (i = 0u; i < MOTOR_AXIS_COUNT; i++)
  {
    if (m == &s_motor_axes[i].handle)
    {
      return &s_motor_axes[i];
    }
  }

  return NULL;
}

void motor_axis_arm(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return;
  }

  motor_driver_arm();
  m->out.pwm_enable = 1u;
}

void motor_axis_disarm(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
  }

  m->out.pwm_enable = 0u;
  motor_driver_disarm();
}

void motor_axis_shutdown(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
  }

  m->out.pwm_enable = 0u;
  motor_driver_shutdown();
}

bool motor_axis_is_armed(const motor_handle_t *m)
{
  (void)m;
  return motor_driver_is_armed();
}

void motor_axis_sync_power(motor_handle_t *m)
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
    if (!motor_axis_is_armed(m))
    {
      motor_axis_arm(m);
    }
  }
  else if (motor_axis_is_armed(m))
  {
    motor_axis_disarm(m);
  }
}

void motor_axis_prepare_dc_cal(motor_handle_t *m)
{
  (void)m;
  motor_driver_prepare_dc_offset_cal();
}

error_t motor_axis_set_pwm(motor_handle_t *m, const motor_actuation_t *actuation)
{
  (void)m;
  return motor_driver_set_pwm(actuation);
}
