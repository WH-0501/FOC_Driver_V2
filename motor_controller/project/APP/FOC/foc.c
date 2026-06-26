#include "foc.h"
#include "foc_motor.h"
#include "current_sense.h"
#include "../config/motor_config.h"
#include <string.h>

motor_handle_t g_motor;

void foc_align_electrical(void);

static void foc_pid_apply_param(pid_state_t *pid, uint8_t mode)
{
  if (pid == NULL)
  {
    return;
  }
  foc_pid_init(pid, mode);
}

void foc_init(const motor_config_t *config)
{
  motor_config_t local_cfg;

  memset(&g_motor, 0, sizeof(g_motor));

  g_motor.ctrl_mode = CTRL_MODE_IDLE;
  g_motor.fsm = STATE_IDLE;
  current_sense_init(&g_motor);

  if (config != NULL)
  {
    g_motor.config = *config;
  }
  else
  {
    motor_config_set_defaults(&local_cfg);
    g_motor.config = local_cfg;
  }

  if (g_motor.config.param.pole_pairs == 0u)
  {
    g_motor.config.param.pole_pairs = 1u;
  }
  if ((g_motor.config.param.direction != 1) && (g_motor.config.param.direction != -1))
  {
    g_motor.config.param.direction = 1;
  }
  if (g_motor.config.param.gear_ratio <= 0.0f)
  {
    g_motor.config.param.gear_ratio = 1.0f;
  }

  foc_pid_apply_param(&g_motor.id_pid, 2u);
  foc_pid_apply_param(&g_motor.iq_pid, 2u);
  foc_pid_apply_param(&g_motor.velocity_pid, 1u);
  foc_pid_apply_param(&g_motor.position_pid, 0u);

  /* LPF：fc 为截止频率 [Hz]，fs = 电流环对 lpf1_update 的调用频率（例 20 kHz） */
  lpf1_init(&g_motor.vbus_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&g_motor.v_a_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&g_motor.v_b_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&g_motor.v_c_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&g_motor.i_d_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&g_motor.i_q_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&g_motor.i_mod_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&g_motor.i_bus_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&g_motor.v_d_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&g_motor.v_q_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);

  lpf1_init(&g_motor.speed_rad_s_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);

  /* 电流零漂校准在上层 current_sense 完成，board 仅负责 ADC raw 采集。 */
  current_sense_calibrate_blocking(&g_motor);

  if (g_motor.config.auto_align_electrical)
  {
    foc_align_electrical();
  }
}

/**
 * @brief 电压控制。设置相电压
 * 将 d、q 轴电压转换为三相电压
 * @param Uq q 轴电压
 * @param Ud d 轴电压
 * @param angle_el 电角度
 */
void foc_voltage(float Uq, float Ud, float angle_el)
{
  float Ualpha, Ubeta;
  float Ua, Ub, Uc;

  // Ud = CLAMP(Ud, -BUS_VOLTAGE * 0.5f, BUS_VOLTAGE * 0.5f);
  // Uq = CLAMP(Uq, -BUS_VOLTAGE * 0.5f, BUS_VOLTAGE * 0.5f);
  float Uq_max = sqrtf(BUS_VOLTAGE * BUS_VOLTAGE - Ud * Ud);
  Uq = CLAMP(Uq, -Uq_max, Uq_max);
  
  inv_park_transform(Ud, Uq, angle_el, &Ualpha, &Ubeta);
  inv_clarke_transform(Ualpha, Ubeta, &Ua, &Ub, &Uc);

  svm(Ua, Ub, Uc, &g_motor.out.duty_a, &g_motor.out.duty_b, &g_motor.out.duty_c);
  g_motor.out.duty_a = CLAMP(g_motor.out.duty_a, 0.0f, 1.0f);
  g_motor.out.duty_b = CLAMP(g_motor.out.duty_b, 0.0f, 1.0f);
  g_motor.out.duty_c = CLAMP(g_motor.out.duty_c, 0.0f, 1.0f);
  set_pwm(&g_motor.out);
}

/**
 * @brief 电流环控制
 * @param id d 轴电流给定
 * @param iq q 轴电流给定
 * @param angle_el 电角度
 * @param phase_vel （预留）电角速度等前馈补偿
 */
void foc_current(float id, float iq, float angle_el, float phase_vel)
{
  float v_d_ctrl = 0.0f;
  float v_q_ctrl = 0.0f;

  // 电流环 PI 控制
  float id_error = id - g_motor.state.i_d;
  float iq_error = iq - g_motor.state.i_q;
  (void)id_error;
  (void)iq_error;
  (void)foc_pid_calc(&g_motor.id_pid, &g_motor.config.pid.id, id, g_motor.state.i_d, &v_d_ctrl);
  (void)foc_pid_calc(&g_motor.iq_pid, &g_motor.config.pid.iq, iq, g_motor.state.i_q, &v_q_ctrl);

  // 电压归一化 = 1/(2/3 * vbus)
  float v_d_norm = 1.5f * g_motor.state.vbus_filtered;
  float v_q_norm = 1.5f * g_motor.state.vbus_filtered;

  g_motor.state.v_d = v_d_ctrl * v_d_norm;
  g_motor.state.v_q = v_q_ctrl * v_q_norm;

  // Vector modulation saturation, lock integrator if saturated
  float factor = 0.9f * SQRT_3_DIV_2 / sqrtf(SQ(g_motor.state.v_d) + SQ(g_motor.state.v_q));
  if (factor < 1.0f) {
    g_motor.state.v_d *= factor;
    g_motor.state.v_q *= factor;
  }

  float alpha, beta;
  float pwm_phase = angle_el + phase_vel * FOC_CURRENT_MEAS_PERIOD;
  inv_park_transform(g_motor.state.v_d, g_motor.state.v_q, g_motor.state.theta_elec_rad, &alpha, &beta);
  
  int result_valid = svpwm(alpha, beta, &g_motor.out.duty_a, &g_motor.out.duty_b, &g_motor.out.duty_c);
  if (result_valid == 0) {
    set_pwm(&g_motor.out);
  }

  g_motor.state.i_bus = sqrtf(SQ(g_motor.state.i_d) + SQ(g_motor.state.i_q));
}

void foc_update(void)
{
  motor_fault_protect(&g_motor);
}

void foc_align_electrical(void)
{
  // 1. 给定相电压,使电机转子吸附到定子绕组上
  foc_voltage(ALIGN_ELECTRICAL_VOLTAGE_Q_V, ALIGN_ELECTRICAL_VOLTAGE_D_V, _3PI_2); // 待验证. 确定是给 d 轴电压还是 q 轴电压
  delay_ms(700);
  foc_voltage(0.0f, 0.0f, _3PI_2); // 先停止电机
  // 2. 读取编码器角度,作为零电角度值
  foc_get_motor_angle();
  // 即编码器零位与电角度零位对齐
  g_motor.config.param.theta_elec_offset_rad = g_motor.state.theta_elec_rad;
}

/**
 * @brief 电流环控制
 * 
 */
#if defined(__ARMCC_VERSION)
__weak void foc_control_loop(void)
#elif defined(__GNUC__)
void foc_control_loop(void) __attribute__((weak))
#else
void foc_control_loop(void)
#endif
{
  /// TODO: 关 TIM Channel4 OC 中断

  // 1. 更新电角度
  foc_get_motor_angle();

  // 2. 获取三相电流并刷新 i_alpha、i_d、i_q
  foc_get_motor_current();

  // 4. 状态机与控制（电压/电流调制等）
  foc_state_machine_loop();

  /// TODO: 开 TIM Channel4 OC 中断
}

void foc_torque_open_control(float target_torque)
{
  // 扭矩开环控制
  float Ud = 0.0f;
  float Uq = 0.5f;

  float ts = 0.001f;
  g_motor.ref.torque_nm = target_torque;
  g_motor.ref.speed_rad_s = g_motor.ref.position_rad + 0.5f * ts;

  foc_voltage(Uq, Ud, g_motor.state.theta_elec_rad);
}

float uq_now = 0.0f;
float omega_now = 0.0f;
float shaft_angle = 0.0f;   /* 电压矢量电角，纯积分，不跟编码器 */
void foc_speed_open_control(float target_speed)
{
#if 0
  float Ud = 0.0f;
  float Uq = 0.5f;

  float ts = 0.001f;
  g_motor.ref.speed_rad_s = target_speed;
  g_motor.ref.position_rad = g_motor.state.position_rad + g_motor.ref.speed_rad_s * ts;

  foc_voltage(Uq, Ud, g_motor.ref.position_rad * g_motor.config.param.pole_pairs);
#else
  const float dt = OPEN_LOOP_TS;
  const float omega_step = OPEN_LOOP_OMEGA_RAMP_RATE * dt;
  // const float uq_step = OPEN_LOOP_UQ_RAMP_RATE * dt;

  omega_now = ramp_toward(omega_now, target_speed, omega_step);
  // uq_now = ramp_toward(uq_now, OL_UQ_MAX/*uq_target_pu*/, uq_step);
  uq_now = OPEN_LOOP_UQ_MAX;
  shaft_angle = angle_normalize(shaft_angle + g_motor.config.param.pole_pairs * omega_now * dt);
  foc_voltage(uq_now, 0.0f, shaft_angle);
#endif
}

void foc_position_open_control(float target_position)
{
  // 位置开环控制
  float Ud = 0.0f;
  float Uq = 0.5f;
  foc_voltage(Uq, Ud, target_position * g_motor.config.param.pole_pairs);
}

void foc_motor_run(void)
{
  // 不同控制模式处理
  switch (g_motor.ctrl_mode) 
  {
    case CTRL_MODE_IDLE:
      break;
    case CTRL_MODE_TORQUE:
      break;
    case CTRL_MODE_SPEED:
      break;
    case CTRL_MODE_POSITION:
      break;
    case CTRL_MODE_CSP:
      break;
    case CTRL_MODE_CSV:
      break;
    case CTRL_MODE_CST:
      break;
    case CTRL_MODE_MIT:
      break;
    case CTRL_MODE_HOMING:
      break;
    case CTRL_MODE_TORQUE_OPEN_LOOP:
      // foc_torque_open_control(g_motor.ref.torque_nm);
      break;
    case CTRL_MODE_SPEED_OPEN_LOOP:
      foc_speed_open_control(g_motor.ref.speed_rad_s);
      break;
    case CTRL_MODE_POSITION_OPEN_LOOP:
      // foc_position_open_control(g_motor.ref.position_rad);
      break;
    case CTRL_MODE_VOLTAGE:
      break;
    case CTRL_MODE_RESERVED:
      break;
    default:
      break;
  }
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
      foc_motor_run();
      break;
    case STATE_FAULT:
      break;
    default:
      break;
  }
}
