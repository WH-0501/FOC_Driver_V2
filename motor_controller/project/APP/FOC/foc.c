#include "foc.h"
#include "foc_sense.h"
#include "foc_fsm.h"
#include "current_sense.h"
#include "foc_pid.h"
#include "motor.h"
#include "../config/motor_config.h"
#include "dwt_profile_delay.h"
#include "common.h"
#include "math_compat.h"
#include <string.h>

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
  if ((m->config.param.direction != 1) && (m->config.param.direction != -1))
  {
    m->config.param.direction = 1;
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
  float u_lim;

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
  float v_d_ctrl = 0.0f;
  float v_q_ctrl = 0.0f;

  (void)phase_vel;

  (void)foc_pid_calc(&m->id_pid, &m->config.pid.id, id, m->state.i_d, &v_d_ctrl);
  (void)foc_pid_calc(&m->iq_pid, &m->config.pid.iq, iq, m->state.i_q, &v_q_ctrl);

  m->state.v_d = v_d_ctrl;
  m->state.v_q = v_q_ctrl;
  foc_voltage(m, v_q_ctrl, v_d_ctrl, angle_el);

  m->state.i_bus = sqrtf(SQ(m->state.i_d) + SQ(m->state.i_q));
}

void foc_align_electrical(motor_handle_t *m)
{
  motor_runtime_t *rt;
  const float align_angle = ALIGN_ELECTRICAL_ANGLE_RAD;
  float theta_locked;

  if (m == NULL)
  {
    return;
  }

  rt = motor_runtime(m);
  if (rt == NULL)
  {
    return;
  }

  /* 对齐前读一次，便于调试对比转子是否被拉动 */
  foc_sense_update(m);
  rt->align_theta_before_rad = m->state.theta_elec_rad;

  motor_arm(m);
  m->out.pwm_enable = 1u;
  foc_voltage(m, ALIGN_ELECTRICAL_VOLTAGE_Q_V, ALIGN_ELECTRICAL_VOLTAGE_D_V, align_angle);
  delay_ms(700);

  /*
   * 必须在对齐电压仍施加时读编码器。
   * 若先置零电压再读，转子会弹回，offset 对应错误位置 → 扭矩模式 dq 解耦失败。
   */
  foc_sense_update(m);
  theta_locked = m->state.theta_elec_rad;
  rt->align_theta_after_rad = theta_locked;

  /*
   * 对齐目标：锁定后 theta_elec == align_angle。
   * theta_elec = raw_elec - offset  =>  offset += theta_elec - align_angle
   */
  m->config.param.theta_elec_offset_rad = angle_normalize(
      m->config.param.theta_elec_offset_rad + theta_locked - align_angle);

  foc_voltage(m, 0.0f, 0.0f, align_angle);
  delay_ms(50);

  (void)foc_pid_clear(&m->id_pid);
  (void)foc_pid_clear(&m->iq_pid);
  rt->electrical_align_done = true;
  motor_disarm(m);
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
  } 
  else 
  {
    rt->open_loop_uq = OPEN_LOOP_UQ_MAX;
    rt->open_loop_shaft_angle = angle_normalize(
        rt->open_loop_shaft_angle + (float)m->config.param.pole_pairs * rt->open_loop_omega * dt);
  }
  
  foc_voltage(m, rt->open_loop_uq, 0.0f, rt->open_loop_shaft_angle);
}

static void foc_speed_closed_control(motor_handle_t *m, float target_speed)
{
  float iq_cmd = 0.0f;

  if (m == NULL)
  {
    return;
  }

  (void)foc_pid_calc(&m->velocity_pid,
                     &m->config.pid.velocity,
                     target_speed,
                     m->state.joint.velocity_rad_s,
                     &iq_cmd);
  foc_current(m, 0.0f, iq_cmd, m->state.theta_elec_rad, 0.0f);
}

static void foc_control_run(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
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
    case CTRL_MODE_SPEED_OPEN_LOOP:
      foc_speed_open_control(m, m->ref.speed_rad_s);
      break;
    default:
      break;
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
    rt->electrical_align_done = true;
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
