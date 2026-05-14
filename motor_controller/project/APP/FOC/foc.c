#include "foc.h"
#include <string.h>

motor_handle_t g_motor;

void foc_init(void)
{
  memset(&g_motor, 0, sizeof(g_motor));

  g_motor.ctrl_mode = CTRL_MODE_IDLE;
  g_motor.fsm = STATE_IDLE;
  g_motor.state.current_calibrating = true;

  // 根据参数进行初始化
  g_motor.config.param.pole_pairs = 1;
  g_motor.config.param.encoder_counts_per_rev = 4096;
  g_motor.config.limits.vbus_nominal = 24.0f;
  g_motor.config.limits.vbus_uv_threshold = 10.0f;
  g_motor.config.limits.id_limit = 30.0f;
  g_motor.config.limits.iq_limit = 30.0f;
  g_motor.config.limits.vd_limit = 12.0f;
  g_motor.config.limits.vq_limit = 12.0f;
  /* sw_overcurrent_trip / hold_ms、vbus_ov_threshold：默认 0，由上位机或工艺写入 */

  g_motor.config.motion.max_speed_rad_s = 300.0f;
  g_motor.config.motion.max_accel_rad_s2 = 1000.0f;
  g_motor.config.motion.max_jerk_rad_s3 = 20000.0f;
  /* snap 未接入规划器时保持 0 */
}

void foc_pwm_start(void)
{
  // 启动 PWM
}

void foc_pwm_stop(void)
{
  // 停止 PWM. 关闭定时器输出
}

void foc_set_pwm(float Ua, float Ub, float Uc)
{
  // 设置 PWM 输出
}

void foc_set_pwm_duty(float duty_a, float duty_b, float duty_c)
{
  // 设置 PWM 占空比
}

void foc_set_pwm_duty(motor_actuation_t *actuation)
{
  // 设置 PWM 占空比
}

/**
 * @brief 电压控制。设置相电压
 * 将 d、q 轴电压转换为三相电压
 * @param Ud d 轴电压
 * @param Uq q 轴电压
 * @param angle_el 电角度
 */
void foc_voltage(float Ud, float Uq, float angle_el)
{
  float Ualpha, Ubeta;
  float Ua, Ub, Uc;

  /* 限幅 */
  Ud = CLAMP(Ud, -BUS_VOLTAGE * 0.5f, BUS_VOLTAGE * 0.5f);
  Uq = CLAMP(Uq, -BUS_VOLTAGE * 0.5f, BUS_VOLTAGE * 0.5f);
  float Uq_max = sqrtf(BUS_VOLTAGE * BUS_VOLTAGE - Ud * Ud);
  Uq = CLAMP(Uq, -Uq_max, Uq_max);
  
  inv_park_transform(Ud, Uq, angle_el, &Ualpha, &Ubeta);
  inv_clarke_transform(Ualpha, Ubeta, &Ua, &Ub, &Uc);

  svpwm(Ua, Ub, Uc, &g_motor.out.duty_a, &g_motor.out.duty_b, &g_motor.out.duty_c);

  set_pwm(&g_motor.out);
}

void foc_update(void)
{
  motor_fault_poll_measurements(&g_motor);
}

#if defined(__ARMCC_VERSION)
__weak void foc_current_loop_control(void)
#elif defined(__GNUC__)
void foc_current_loop_control(void) __attribute__((weak))
#else
void foc_current_loop_control(void)
#endif
{
  get_phase_current();
  board_current_offset_cal_fsm_step(&g_motor);
}

void foc_next_state(fsm_state_t next_state)
{
  if (next_state >= STATE_MAX)
  {
    return;
  }

  g_motor.fsm = next_state;
}

void foc_state_machine_loop(void)
{
  switch (g_motor.fsm)
  {
    case STATE_IDLE:
      g_motor.ctrl_mode = CTRL_MODE_IDLE;
      /* 关闭所有 PWM 通道 */

      /* 清除 PID 状态 PID_clear */

      /* 重置 FOC. 清除 i_d、i_q 等 */

      foc_next_state(STATE_STARTUP);
      break;
    case STATE_STARTUP:
      /* 零漂校准 */

      foc_next_state(STATE_ENCODER_CALIBRATION);
      break;
    case STATE_ENCODER_CALIBRATION:
      ///< TODO:
      foc_next_state(STATE_RSLS_CALIBRATION);
      break;
    case STATE_RSLS_CALIBRATION:
      ///< TODO:
      foc_next_state(STATE_FLUX_CALIBRATION);
      break;
    case STATE_FLUX_CALIBRATION:
      ///< TODO:
      foc_next_state(STATE_ELECTRICAL_ALIGNMENT);
      break;
    case STATE_ELECTRICAL_ALIGNMENT:
      ///< TODO:
      foc_next_state(STATE_ANTICOGGING);
      break;
    case STATE_ANTICOGGING:
      ///< TODO:
      foc_next_state(STATE_RUNNING);
      break;
    case STATE_RUNNING:

      break;
    case STATE_FAULT:
      break;
    default:
      break;
  }
}
