#include "foc.h"
#include "foc_motor.h"
#include "foc_fsm.h"
#include "current_sense.h"
#include "motor_axis.h"
#include "../config/motor_config.h"
#include <string.h>

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
  motor_axis_t *axis = motor_axis_get(0u);
  motor_handle_t *m = &axis->handle;

  memset(m, 0, sizeof(*m));
  axis->electrical_align_done = false;
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

  foc_pid_apply_param(&m->id_pid, 2u);
  foc_pid_apply_param(&m->iq_pid, 2u);
  foc_pid_apply_param(&m->velocity_pid, 1u);
  foc_pid_apply_param(&m->position_pid, 0u);

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
  lpf1_init(&m->speed_rad_s_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);

  current_sense_init(m);

  if (m->config.auto_align_electrical)
  {
    foc_align_electrical();
    axis->electrical_align_done = true;
  }
}

void foc_voltage(float Uq, float Ud, float angle_el)
{
  float Ualpha, Ubeta;
  float Ua, Ub, Uc;
  motor_handle_t *m = &g_motor;

  float Uq_max = sqrtf(BUS_VOLTAGE * BUS_VOLTAGE - Ud * Ud);
  Uq = CLAMP(Uq, -Uq_max, Uq_max);

  inv_park_transform(Ud, Uq, angle_el, &Ualpha, &Ubeta);
  inv_clarke_transform(Ualpha, Ubeta, &Ua, &Ub, &Uc);

  svm(Ua, Ub, Uc, &m->out.duty_a, &m->out.duty_b, &m->out.duty_c);
  m->out.duty_a = CLAMP(m->out.duty_a, 0.0f, 1.0f);
  m->out.duty_b = CLAMP(m->out.duty_b, 0.0f, 1.0f);
  m->out.duty_c = CLAMP(m->out.duty_c, 0.0f, 1.0f);
  (void)motor_axis_set_pwm(m, &m->out);
}

void foc_current(float id, float iq, float angle_el, float phase_vel)
{
  float v_d_ctrl = 0.0f;
  float v_q_ctrl = 0.0f;
  motor_handle_t *m = &g_motor;
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
    (void)motor_axis_set_pwm(m, &m->out);
  }

  m->state.i_bus = sqrtf(SQ(m->state.i_d) + SQ(m->state.i_q));
}

void foc_align_electrical(void)
{
  motor_handle_t *m = &g_motor;

  motor_axis_arm(m);
  foc_voltage(ALIGN_ELECTRICAL_VOLTAGE_Q_V, ALIGN_ELECTRICAL_VOLTAGE_D_V, _3PI_2);
  delay_ms(700);
  foc_voltage(0.0f, 0.0f, _3PI_2);
  foc_get_motor_angle();
  m->config.param.theta_elec_offset_rad = m->state.theta_elec_rad;
  motor_axis_disarm(m);
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

  foc_get_motor_angle();
  foc_get_motor_current();
  foc_fsm_loop(m);

  if (m->fsm == STATE_RUNNING)
  {
    foc_motor_run();
  }
}

void foc_torque_open_control(float target_torque)
{
  float Ud = 0.0f;
  float Uq = 0.5f;
  float ts = 0.001f;
  motor_handle_t *m = &g_motor;

  m->ref.torque_nm = target_torque;
  m->ref.speed_rad_s = m->ref.position_rad + 0.5f * ts;
  foc_voltage(Uq, Ud, m->state.theta_elec_rad);
}

float uq_now = 0.0f;
float omega_now = 0.0f;
float shaft_angle = 0.0f;

void foc_speed_open_control(float target_speed)
{
  const float dt = OPEN_LOOP_TS;
  const float omega_step = OPEN_LOOP_OMEGA_RAMP_RATE * dt;
  motor_handle_t *m = &g_motor;

  omega_now = ramp_toward(omega_now, target_speed, omega_step);
  uq_now = OPEN_LOOP_UQ_MAX;
  shaft_angle = angle_normalize(shaft_angle + m->config.param.pole_pairs * omega_now * dt);
  foc_voltage(uq_now, 0.0f, shaft_angle);
}

void foc_position_open_control(float target_position)
{
  float Ud = 0.0f;
  float Uq = 0.5f;
  motor_handle_t *m = &g_motor;

  foc_voltage(Uq, Ud, target_position * m->config.param.pole_pairs);
}

void foc_motor_run(void)
{
  motor_handle_t *m = &g_motor;

  switch (m->ctrl_mode)
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
      break;
    case CTRL_MODE_SPEED_OPEN_LOOP:
      foc_speed_open_control(m->ref.speed_rad_s);
      break;
    case CTRL_MODE_POSITION_OPEN_LOOP:
      break;
    case CTRL_MODE_VOLTAGE:
      break;
    case CTRL_MODE_RESERVED:
      break;
    default:
      break;
  }
}
