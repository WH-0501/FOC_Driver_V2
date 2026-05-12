#include "foc.h"
#include <string.h>

motor_handle_t g_motor;

void foc_init(void)
{
  memset(&g_motor, 0, sizeof(g_motor));

  g_motor.config.param.pole_pairs = 1;
  g_motor.config.param.encoder_counts_per_rev = 4096;
  g_motor.config.limits.vbus_nominal = 24.0f;
  g_motor.config.limits.vbus_uv_threshold = 10.0f;
  g_motor.config.limits.id_limit = 30.0f;
  g_motor.config.limits.iq_limit = 30.0f;
  g_motor.config.limits.vd_limit = 12.0f;
  g_motor.config.limits.vq_limit = 12.0f;
  /* sw_overcurrent_trip / hold_ms、vbus_ov_threshold：默认 0，由上位机或工艺写入 */

  g_motor.config.motion.omega_max_abs_mech_rad_s = 300.0f;
  g_motor.config.motion.alpha_max_abs_mech_rad_s2 = 1000.0f;
  g_motor.config.motion.jerk_max_abs_mech_rad_s3 = 20000.0f;
  /* snap 未接入规划器时保持 0 */
}

void foc_update(void)
{
  motor_fault_poll_measurements(&g_motor);
}

#if defined(__ARMCC_VERSION)
__weak void foc_current_loop_control(void)
#else
__attribute__((weak)) void foc_current_loop_control(void)
#endif
{
    get_phase_current();
}

void foc_state_machint_loop(void)
{

}
