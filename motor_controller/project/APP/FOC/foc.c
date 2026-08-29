#include "foc.h"
#include "foc_sense.h"
#include "foc_fsm.h"
#include "current_sense.h"
#include "motor.h"
#include "../config/motor_config.h"
#include "dwt_profile_delay.h"
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
  lpf1_init(&m->vbus_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
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
  float vbus = m->config.limits.vbus_nominal;
  float Uq_max;

  if (vbus <= 0.0f)
  {
    vbus = BUS_VOLTAGE;
  }

  Uq_max = sqrtf(vbus * vbus - Ud * Ud);
  Uq = CLAMP(Uq, -Uq_max, Uq_max);

  inv_park_transform(Ud, Uq, angle_el, &Ualpha, &Ubeta);
  inv_clarke_transform(Ualpha, Ubeta, &Ua, &Ub, &Uc);

  svm(Ua, Ub, Uc, &m->out.duty_a, &m->out.duty_b, &m->out.duty_c);
  m->out.duty_a = CLAMP(m->out.duty_a, 0.0f, 1.0f);
  m->out.duty_b = CLAMP(m->out.duty_b, 0.0f, 1.0f);
  m->out.duty_c = CLAMP(m->out.duty_c, 0.0f, 1.0f);
  (void)motor_apply_pwm(m, &m->out);
}

static void foc_current(motor_handle_t *m, float id, float iq, float angle_el, float phase_vel)
{
  float v_d_ctrl = 0.0f;
  float v_q_ctrl = 0.0f;
  float alpha, beta;
  int result_valid;

  (void)angle_el;
  (void)phase_vel;

  (void)foc_pid_calc(&m->id_pid, &m->config.pid.id, id, m->state.i_d, &v_d_ctrl);
  (void)foc_pid_calc(&m->iq_pid, &m->config.pid.iq, iq, m->state.i_q, &v_q_ctrl);

  {
    float v_d_norm = 1.5f * m->state.vbus_filtered;
    float v_q_norm = 1.5f * m->state.vbus_filtered;
    (void)v_q_norm;

    m->state.v_d = v_d_ctrl * v_d_norm;
    m->state.v_q = v_q_ctrl * v_d_norm;
  }

  {
    float factor = 0.9f * SQRT_3_DIV_2 / sqrtf(SQ(m->state.v_d) + SQ(m->state.v_q));
    if (factor < 1.0f)
    {
      m->state.v_d *= factor;
      m->state.v_q *= factor;
    }
  }

  inv_park_transform(m->state.v_d, m->state.v_q, m->state.theta_elec_rad, &alpha, &beta);

  result_valid = svpwm(alpha, beta, &m->out.duty_a, &m->out.duty_b, &m->out.duty_c);
  if (result_valid == 0)
  {
    (void)motor_apply_pwm(m, &m->out);
  }

  m->state.i_bus = sqrtf(SQ(m->state.i_d) + SQ(m->state.i_q));
}

static void foc_align_electrical(motor_handle_t *m)
{
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

  motor_arm(m);
  foc_voltage(m, ALIGN_ELECTRICAL_VOLTAGE_Q_V, ALIGN_ELECTRICAL_VOLTAGE_D_V, _3PI_2);
  delay_ms(700);
  foc_voltage(m, 0.0f, 0.0f, _3PI_2);
  foc_sense_update(m);
  m->config.param.theta_elec_offset_rad = m->state.theta_elec_rad;
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
  rt->open_loop_uq = OPEN_LOOP_UQ_MAX;
  rt->open_loop_shaft_angle = angle_normalize(
      rt->open_loop_shaft_angle + (float)m->config.param.pole_pairs * rt->open_loop_omega * dt);
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

  foc_sense_update(m);
  foc_fsm_loop(m);

  if (m->fsm == STATE_RUNNING)
  {
    foc_control_run(m);
  }
}
