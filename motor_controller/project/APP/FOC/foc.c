#include "foc.h"
#include "foc_sense.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "current_sense.h"
#include "foc_pid.h"
#include "motor.h"
#include "../config/motor_config.h"
#include "dwt_profile_delay.h"
#include "common.h"
#include "math_compat.h"
#include <string.h>

/**
 * @brief 计算时间差
 * 
 * @param t0_us: 开始时间
 * @return uint32_t: 时间差[us]
 */
static uint32_t foc_dwt_elapsed_us(uint32_t t0_us)
{
  return (uint32_t)(dwt_get_ticks_us() - t0_us);
}

static void foc_pid_apply_param(pid_state_t *pid, uint8_t mode)
{
  if (pid == NULL)
  {
    return;
  }
  foc_pid_init(pid, mode);
}

static void foc_validate_config(motor_handle_t *m)
{
  if (m->config.param.pole_pairs == 0u)
  {
    m->config.param.pole_pairs = 1u;
  }
  if ((m->config.param.direction != DIR_CW) && (m->config.param.direction != DIR_CCW))
  {
    m->config.param.direction = DIR_CW;
  }
  if (m->config.param.gear_ratio <= 0.0f)
  {
    m->config.param.gear_ratio = 1.0f;
  }
}

static void foc_init_filters(motor_handle_t *m)
{
  lpf1_init(&m->vbus_lpf, FOC_VBUS_LPF_FC_HZ, FOC_VBUS_SAMPLE_FS_HZ);
  lpf1_init(&m->v_a_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&m->v_b_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&m->v_c_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&m->i_d_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&m->i_q_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&m->i_mod_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&m->i_bus_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&m->v_d_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
  lpf1_init(&m->v_q_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
}

static void foc_voltage(motor_handle_t *m, float Uq, float Ud, float angle_el)
{
  float Ualpha, Ubeta;
  float Ua, Ub, Uc;
  float svm_a, svm_b, svm_c;
  // float vbus = m->config.limits.vbus_nominal;
  float vbus;
  // float Uq_max;
  // float u_lim;

  vbus = m->state.vbus_filtered;
  if (vbus <= 0.0f)
  {
    vbus = m->config.limits.vbus_nominal;
  }
  if (vbus <= 0.0f)
  {
    vbus = BUS_VOLTAGE;
  }

  // 限幅方案一
  // u_lim = 0.5f * vbus;
  // Uq = CLAMP(Uq, -u_lim, u_lim);
  // Ud = CLAMP(Ud, -u_lim, u_lim);
  // Uq_max = sqrtf(vbus * vbus - Ud * Ud);
  // Uq = CLAMP(Uq, -Uq_max, Uq_max);

  inv_park_transform(Ud, Uq, angle_el, &Ualpha, &Ubeta);
  m->inv_park_result.u_alpha = Ualpha;
  m->inv_park_result.u_beta = Ubeta;

  // 限幅方案二
  float Vm, Vmax;
  Vm = sqrtf(SQ(Ualpha) + SQ(Ubeta));
  Vmax = vbus * ONE_DIV_SQRT_3;
  if (Vm > Vmax)
  {
    float k = Vmax / Vm;
    Ualpha *= k;
    Ubeta *= k;
  }

  inv_clarke_transform(Ualpha, Ubeta, &Ua, &Ub, &Uc);
  m->inv_clarke_result.u_a = Ua;
  m->inv_clarke_result.u_b = Ub;
  m->inv_clarke_result.u_c = Uc;

  svm(Ua, Ub, Uc, &svm_a, &svm_b, &svm_c);
  m->out.duty_a = CLAMP(0.5f + svm_a / vbus, 0.0f, 1.0f);
  m->out.duty_b = CLAMP(0.5f + svm_b / vbus, 0.0f, 1.0f);
  m->out.duty_c = CLAMP(0.5f + svm_c / vbus, 0.0f, 1.0f);
  (void)motor_apply_pwm(m, &m->out);
}

static void foc_current(motor_handle_t *m, float id, float iq, float angle_el, float phase_vel)
{
  motor_runtime_t *rt = motor_runtime(m);
  float v_d_ctrl = 0.0f;
  float v_q_ctrl = 0.0f;
  uint32_t t0_us = dwt_get_ticks_us();

  (void)phase_vel;

  (void)foc_pid_calc(&m->id_pid, &m->config.pid.id, id, m->state.i_d, &v_d_ctrl);
  (void)foc_pid_calc(&m->iq_pid, &m->config.pid.iq, iq, m->state.i_q, &v_q_ctrl);

  m->state.v_d = v_d_ctrl;
  m->state.v_q = v_q_ctrl;
  foc_voltage(m, v_q_ctrl, v_d_ctrl, angle_el);

  m->state.i_bus = sqrtf(SQ(m->state.i_d) + SQ(m->state.i_q));

  if (rt != NULL)
  {
    rt->foc_cur_loop_us = foc_dwt_elapsed_us(t0_us);
  }
}

/** 编码器 raw 电角（不含 theta_elec_offset），与 encoder_publish_state 一致 */
static float foc_align_raw_elec_rad(const motor_handle_t *m, int8_t dir)
{
  float pole_pairs;

  (void)dir;

  if (m == NULL) return 0.0f;

  pole_pairs = (m->config.param.pole_pairs > 0u) ? (float)m->config.param.pole_pairs : 1.0f;
  return m->state.shaft.angle_rad * pole_pairs;
}

static void foc_align_hold_voltage(motor_handle_t *m, float ud, float uq, float angle_el, uint32_t hold_ms)
{
  uint32_t elapsed = 0u;

  if (m == NULL) return;

  while (elapsed < hold_ms)
  {
    foc_voltage(m, uq, ud, angle_el);
    delay_ms(ALIGN_SAMPLE_INTERVAL_MS);
    elapsed += ALIGN_SAMPLE_INTERVAL_MS;
  }
}

static float foc_align_sample_raw_elec(motor_handle_t *m,
                                       int8_t dir,
                                       float ud,
                                       float uq,
                                       float angle_el,
                                       uint8_t count,
                                       float *spread_out)
{
  float sum = 0.0f;
  float raw, raw_min = 1e9f, raw_max = -1e9f;

  if ((m == NULL) || (count == 0u)) return 0.0f;
  if (spread_out != NULL) { *spread_out = 0.0f; }

  for (uint8_t i = 0u; i < count; i++)
  {
    foc_voltage(m, uq, ud, angle_el);
    foc_sense_update(m);
    raw = foc_align_raw_elec_rad(m, dir);
    sum += raw;
    if (raw < raw_min) raw_min = raw;
    if (raw > raw_max) raw_max = raw;
    delay_ms(ALIGN_SAMPLE_INTERVAL_MS);
  }

  /* 计算分散度 */
  if (spread_out != NULL) 
  {
    float range = raw_max - raw_min;
    if (range > _PI) { range = _2PI - range; } 
    *spread_out = range; 
  }

  return sum / (float)count;
}

static float foc_align_probe_iq(motor_handle_t *m, float uq, float ud, float angle_el, uint16_t hold_ms)
{
  if (m == NULL) return 0.0f;

  foc_align_hold_voltage(m, ud, uq, angle_el, hold_ms);
  foc_sense_update(m);
  return m->state.i_q;
}

/* 转子吸合验证：在 align_angle 的反相点（±π）二次锁定，
 * 确认转子确实跟随磁场转动了约 π 电角度且稳定到位。
 *  - 堵转/负载大于吸合力：转子停在初次锁定点，位移≈0 → 失败；
 *  - 振荡/爬行未稳定：二次采样分散度超限 → 失败；
 *  - 注意：±π 在 mod 2π 下不可分辨方向，N/S 极性交由 +Vq 探针负责。
 * 返回 1=吸合验证通过，0=失败；raw_engage_out/spread_out 为诊断输出（失败时仍有效）。 */
 static uint8_t foc_align_check_rotor_engaged(motor_handle_t *m,
                                              int8_t dir,
                                              float ud,
                                              float align_angle,
                                              float raw_locked,
                                              uint8_t count,
                                              float *raw_engage_out,
                                              float *spread_out)
{
  const float engage_angle = angle_normalize(align_angle + _PI);
  float raw_engage;
  float spread;
  float delta;

  if (raw_engage_out != NULL)
  {
    *raw_engage_out = 0.0f;
  }
  if (spread_out != NULL)
  {
    *spread_out = 0.0f;
  }
  if (m == NULL)
  {
    return 0u;
  }

  /* 反相点二次锁定（u_q 置零，沿用翻转路径的 ALIGN_RELOCK_MS 沉降时间） */
  foc_align_hold_voltage(m, ud, 0.0f, engage_angle, ALIGN_RELOCK_MS);

  /* 采样二次锁定位置与分散度 */
  raw_engage = foc_align_sample_raw_elec(m, dir, ud, 0.0f, engage_angle, count, &spread);

  /* 输出诊断（无论成败） */
  if (raw_engage_out != NULL)
  {
    *raw_engage_out = raw_engage;
  }
  if (spread_out != NULL)
  {
    *spread_out = spread;
  }

  /* 位移检查：圆上距离应 ≈ π（堵转 ≈ 0） */
  delta = fabsf(angle_normalize(raw_engage - raw_locked));
  if (delta < _PI - ALIGN_ROTOR_ENGAGE_TOL_RAD)
  {
    return 0u;
  }

  /* 稳定检查：二次锁定采样分散度过大 → 未吸合到位/振荡 */
  if (spread > ALIGN_ROTOR_SPREAD_MAX_RAD)
  {
    return 0u;
  }

  return 1u;
}


error_t foc_align_electrical(motor_handle_t *m)
{
  error_t ret = ERR_NONE;
  motor_runtime_t *rt;
  const float align_angle = ALIGN_ELECTRICAL_ANGLE_RAD;
  const float ud = ALIGN_ELECTRICAL_VOLTAGE_D_V;
  float raw_locked = 0.0f;
  float raw_engage_out = 0.0f;
  float spread_out = 0.0f;
  float iq_probe = 0.0f;
  int8_t dir;

  if (m == NULL)
  {
    return ERR_NULL;
  }

  rt = motor_runtime(m);
  if (rt == NULL)
  {
    return ERR_NULL;
  }

  dir = m->config.param.direction;
  rt->align_direction_flipped = false;
  rt->align_ok = false;
  rt->align_engaged = false;

  foc_sense_update(m);
  rt->align_theta_before_rad = m->state.theta_elec_rad;

  motor_arm(m);
  m->out.pwm_enable = 1u;

  /** 
   * 在执行 D 轴锁定时，强制转子物理位置停在 align_angle 位置，此时 theta_elec 必须等于 align_angle.
   * 
   * 重复调用 foc_voltage 锁定原因是：PWM输出就会消失，转子失去锁定力，可能会因外界扰动或自身磁阻而轻微偏移
   */

  /* 1) d 轴锁定：固定 open-loop 电角下施加 Ud */
  foc_align_hold_voltage(m, ud, ALIGN_ELECTRICAL_VOLTAGE_Q_V, align_angle, ALIGN_HOLD_MS);

  /* 2) 加压期间多次采样 raw 电角，写 offset */
  raw_locked = foc_align_sample_raw_elec(m,
                                       dir,
                                       ud,
                                       ALIGN_ELECTRICAL_VOLTAGE_Q_V,
                                       align_angle,
                                       (uint8_t)ALIGN_SAMPLE_COUNT,
                                       &spread_out);
  /* 锁定稳定检查: 采样分散度过大,转子吸合不到位/振荡，offset不可信 */
  if (spread_out > ALIGN_ROTOR_SPREAD_MAX_RAD)
  {
    motor_fault_raise(m, FAULT_ALIGN_FAILED);
    ret = ERR_ALIGIN_SPREAD_OUT;
    goto exit;
  }
  rt->align_raw_elec_rad = raw_locked;

  /* 3) +Vq 极性探针：正 Vq 应产生正 i_q（仍保持 d 轴锁定角 align_angle 作 Park 角） */
  iq_probe = foc_align_probe_iq(m, ALIGN_POLARITY_TEST_UQ_V, ud, align_angle, ALIGN_POLARITY_TEST_MS);

  if (fabsf(iq_probe) >= ALIGN_POLARITY_IQ_MIN_A && iq_probe < 0.0f)
  {
    /* 4) 翻转 encoder direction 并在同一锁定位置重算 offset（而非 offset+=π） */
    if (m->config.param.direction == DIR_CCW)
    {
      m->config.param.direction = DIR_CW;
    }
    else
    {
      m->config.param.direction = DIR_CCW;
    }
    dir = m->config.param.direction;
    rt->align_direction_flipped = true;

    foc_align_hold_voltage(m, ud, 0.0f, align_angle, ALIGN_RELOCK_MS);
    raw_locked = foc_align_sample_raw_elec(m, dir, ud, 0.0f, align_angle, 
                  (uint8_t)ALIGN_SAMPLE_COUNT, &spread_out);
    rt->align_raw_elec_rad = raw_locked;
    if (spread_out > ALIGN_ROTOR_SPREAD_MAX_RAD)
    {
      motor_fault_raise(m, FAULT_ALIGN_FAILED);
      ret = ERR_ALIGIN_SPREAD_OUT;
      goto exit;
    }
    iq_probe = foc_align_probe_iq(m, ALIGN_POLARITY_TEST_UQ_V, ud, align_angle, ALIGN_POLARITY_TEST_MS);
  }

  /* 5) 极性探针可能扰动转子：吸合验证前重新 d 轴锁定并采样 baseline */
  foc_align_hold_voltage(m, ud, 0.0f, align_angle, ALIGN_RELOCK_MS);
  raw_locked = foc_align_sample_raw_elec(m, dir, ud, 0.0f, align_angle,
                                         (uint8_t)ALIGN_SAMPLE_COUNT, &spread_out);
  if (spread_out > ALIGN_ROTOR_SPREAD_MAX_RAD)
  {
    motor_fault_raise(m, FAULT_ALIGN_FAILED);
    ret = ERR_ALIGIN_SPREAD_OUT;
    goto exit;
  }
  rt->align_raw_elec_rad = raw_locked;

  /* 6) 转子吸合验证 */
  rt->align_engaged = foc_align_check_rotor_engaged(m, dir, ud, align_angle, raw_locked, 
                                                    (uint8_t)ALIGN_SAMPLE_COUNT, 
                                                    &raw_engage_out, &spread_out);
  rt->align_raw_engage_rad = raw_engage_out; // 转子吸合采样 raw 电角（未减 offset）
  if (!rt->align_engaged)
  {
    motor_fault_raise(m, FAULT_ALIGN_FAILED);
    ret = ERR_ALIGIN_ROTOR_NOT_ENGAGED;
    goto exit;
  }

  rt->align_polarity_test_iq = iq_probe;
  if (fabsf(iq_probe) >= ALIGN_POLARITY_IQ_MIN_A && iq_probe > 0.0f)
  {
    rt->align_ok = true;
  }
  else if (fabsf(iq_probe) < ALIGN_POLARITY_IQ_MIN_A)
  {
    /* 电流过小：仍视为 offset 步骤完成，但极性未验证 */
    rt->align_ok = true;
  }

  foc_voltage(m, 0.0f, 0.0f, align_angle);
  delay_ms(50);

  (void)foc_pid_clear(&m->id_pid);
  (void)foc_pid_clear(&m->iq_pid);
  rt->electrical_align_done = rt->align_ok;
  if (rt->align_ok)
  {
    /* theta_elec = normalize(raw - offset)；锁定时应等于 align_angle */
    m->config.param.theta_elec_offset_rad = angle_normalize(rt->align_raw_elec_rad - align_angle);
    if ((m->config.param.direction != DIR_CW) && (m->config.param.direction != DIR_CCW))
    {
      m->config.param.direction = DIR_CW;
    }
    foc_sense_update(m);
    rt->align_theta_after_rad = m->state.theta_elec_rad;
    rt->open_loop_elec_angle_rad = m->state.theta_elec_rad;
  }
  if (!rt->align_ok)
  {
    motor_fault_raise(m, FAULT_ALIGN_FAILED);
    ret = ERR_ALIGIN_FAILED;
  }

exit:
  motor_disarm(m);
  return ret;
}

static void foc_speed_open_control(motor_handle_t *m, float target_speed)
{
  motor_runtime_t *rt;
  const float dt = OPEN_LOOP_TS;
  const float omega_step = OPEN_LOOP_OMEGA_RAMP_RATE * dt;

  if (m == NULL)
  {
    return;
  }

  rt = motor_runtime(m);
  if (rt == NULL)
  {
    return;
  }

  rt->open_loop_omega = ramp_toward(rt->open_loop_omega, target_speed, omega_step);

  if ((fabsf(target_speed) < OPEN_LOOP_OMEGA_EPS_RAD_S) &&
      (fabsf(rt->open_loop_omega) < OPEN_LOOP_OMEGA_EPS_RAD_S))
  {
    rt->open_loop_uq = 0.0f;
    rt->open_loop_iq = 0.0f;
  }
  else
  {
    rt->open_loop_uq = OPEN_LOOP_UQ_MAX;
    // rt->open_loop_iq = OPEN_LOOP_IQ_MAX;
    /* ref.speed / open_loop_omega 为关节端 [rad/s]；电角速率 = pp × ω_shaft = pp × ω_joint × gear_ratio */
    {
      float gear_ratio = (m->config.param.gear_ratio > 0.0f) ? m->config.param.gear_ratio : 1.0f;
      rt->open_loop_elec_angle_rad = angle_normalize(
          rt->open_loop_elec_angle_rad
          + (float)m->config.param.pole_pairs * rt->open_loop_omega * gear_ratio * dt);
    }
  }

  foc_voltage(m, rt->open_loop_uq, 0.0f, rt->open_loop_elec_angle_rad);
  // foc_current(m, 0.0f, rt->open_loop_iq, rt->open_loop_elec_angle_rad, 0.0f);
}

static void foc_speed_closed_control(motor_handle_t *m, float target_speed)
{
  motor_runtime_t *rt = motor_runtime(m);
  float iq_cmd = 0.0f;
  uint32_t t0_us;

  if (m == NULL)
  {
    return;
  }

  t0_us = dwt_get_ticks_us();
  (void)foc_pid_calc(&m->velocity_pid,
                     &m->config.pid.velocity,
                     target_speed,
                     m->state.joint.velocity_rad_s,
                     &iq_cmd);
  if (rt != NULL)
  {
    rt->foc_vel_loop_us = foc_dwt_elapsed_us(t0_us);
  }

  foc_current(m, 0.0f, iq_cmd, m->state.theta_elec_rad, 0.0f);
}


static void foc_position_closed_control(motor_handle_t *m, float target_pos)
{
  motor_runtime_t *rt = motor_runtime(m);
  float speed_cmd = 0.0f;
  uint32_t t0_us;
  if (m == NULL)
  {
    return;
  }
  t0_us = dwt_get_ticks_us();
  (void)foc_pid_calc(&m->position_pid,
                     &m->config.pid.position,
                     target_pos,
                     m->state.joint.position_rad,
                     &speed_cmd);
  if (rt != NULL)
  {
    rt->foc_pos_loop_us = foc_dwt_elapsed_us(t0_us);
  }
  if (m->config.motion.max_speed_rad_s > 0.0f)
  {
    speed_cmd = CLAMP(speed_cmd,
                      -m->config.motion.max_speed_rad_s,
                      m->config.motion.max_speed_rad_s);
  }

  foc_speed_closed_control(m, speed_cmd);
}

static void foc_control_run(motor_handle_t *m)
{
  motor_runtime_t *rt = motor_runtime(m);
  uint32_t t_ctrl_us;

  if (m == NULL)
  {
    return;
  }

  t_ctrl_us = dwt_get_ticks_us();
  if (rt != NULL)
  {
    rt->foc_pos_loop_us = 0.0f;
    rt->foc_vel_loop_us = 0.0f;
    rt->foc_cur_loop_us = 0.0f;
  }

  switch (m->ctrl_mode)
  {
    case CTRL_MODE_TORQUE:
    {
      float iq_limit = m->config.limits.iq_limit;
      float iq_ref = m->ref.iq;

      if (iq_limit > 0.0f)
      {
        iq_ref = CLAMP(iq_ref, -iq_limit, iq_limit);
      }
      foc_current(m, 0.0f, iq_ref, m->state.theta_elec_rad, 0.0f);
      break;
    }
    case CTRL_MODE_SPEED:
      foc_speed_closed_control(m, m->ref.speed_rad_s);
      break;
    case CTRL_MODE_POSITION:
      foc_position_closed_control(m, m->ref.position_rad);
      break;
    case CTRL_MODE_SPEED_OPEN_LOOP:
      foc_speed_open_control(m, m->ref.speed_rad_s);
      break;
    default:
      break;
  }

  if (rt != NULL)
  {
    rt->foc_ctrl_loop_us = foc_dwt_elapsed_us(t_ctrl_us);
  }
}

void foc_init(motor_handle_t *m, const motor_config_t *config)
{
  motor_config_t local_cfg;
  motor_runtime_t *rt;

  if (m == NULL)
  {
    return;
  }

  rt = motor_runtime(m);
  if (rt == NULL)
  {
    return;
  }

  memset(m, 0, sizeof(*m));
  memset(rt, 0, sizeof(*rt));
  rt->electrical_align_done = false;
  foc_fsm_init(m);

  if (config != NULL)
  {
    m->config = *config;
  }
  else
  {
    motor_config_set_defaults(&local_cfg);
    m->config = local_cfg;
  }

  foc_validate_config(m);

  foc_pid_apply_param(&m->id_pid, 2u);
  foc_pid_apply_param(&m->iq_pid, 2u);
  foc_pid_apply_param(&m->velocity_pid, 1u);
  foc_pid_apply_param(&m->position_pid, 0u);
  foc_init_filters(m);

  if (m->config.limits.vbus_nominal > 0.0f)
  {
    m->state.vbus = m->config.limits.vbus_nominal;
    m->state.vbus_filtered = m->config.limits.vbus_nominal;
    lpf1_reset(&m->vbus_lpf, m->state.vbus);
  }

  current_sense_init(m);

  if (m->config.auto_align_electrical)
  {
    foc_align_electrical(m);
  }
}

#if defined(__ARMCC_VERSION)
__weak void foc_control_loop(motor_handle_t *m)
#elif defined(__GNUC__)
void foc_control_loop(motor_handle_t *m) __attribute__((weak))
#else
void foc_control_loop(motor_handle_t *m)
#endif
{
  if (m == NULL)
  {
    return;
  }

  // foc_sense_poll_vbus(m);
  foc_sense_update(m);
  foc_fsm_loop(m);

  if (m->fsm == STATE_RUNNING)
  {
    foc_control_run(m);
  }
}
