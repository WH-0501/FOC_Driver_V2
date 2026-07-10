#include "motor_driver.h"
#include "board.h"
#include "gate/gate_driver.h"

static bool s_motor_driver_armed;

static void motor_driver_apply_disarm(void)
{
  const gate_driver_t *drv = gate_driver_get_active();

  (void)pwm_hw_stop();
  (void)pwm_hw_lowside_brake_off();
  gate_driver_exit_sleep(drv);
  gate_driver_set_all_phase_en(drv, false);
  (void)current_hw_init();
  s_motor_driver_armed = false;
}

static void motor_driver_apply_arm(void)
{
  const gate_driver_t *drv = gate_driver_get_active();

  (void)pwm_hw_init();
  (void)pwm_hw_lowside_brake_off();
  gate_driver_exit_sleep(drv);
  gate_driver_set_all_phase_en(drv, true);
  (void)pwm_hw_start();
  (void)current_hw_init();
  s_motor_driver_armed = true;
}

void motor_driver_init(void)
{
  s_motor_driver_armed = false;
  motor_driver_shutdown();
}

void motor_driver_arm(void)
{
  if (s_motor_driver_armed)
  {
    return;
  }

  motor_driver_apply_arm();
}

void motor_driver_disarm(void)
{
  motor_driver_apply_disarm();
}

void motor_driver_shutdown(void)
{
  const gate_driver_t *drv = gate_driver_get_active();

  (void)pwm_hw_stop();
  (void)pwm_hw_lowside_brake_off();
  gate_driver_enter_inactive_state(drv);
  (void)current_hw_deinit();
  s_motor_driver_armed = false;
}

bool motor_driver_is_armed(void)
{
  return s_motor_driver_armed;
}

void motor_driver_prepare_dc_offset_cal(void)
{
  const gate_driver_t *drv = gate_driver_get_active();

  (void)current_hw_deinit();
  (void)pwm_hw_init();
  gate_driver_exit_sleep(drv);
  gate_driver_set_all_phase_en(drv, true);
  (void)pwm_hw_lowside_brake_on();
  (void)pwm_hw_start();
  s_motor_driver_armed = false;
}

error_t motor_driver_set_pwm(const motor_actuation_t *actuation)
{
  if (!s_motor_driver_armed)
  {
    return ERR_FAIL;
  }

  if (actuation == NULL)
  {
    return ERR_FAIL;
  }

  return set_pwm((motor_actuation_t *)actuation);
}
