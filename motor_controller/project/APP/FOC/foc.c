#include "foc.h"
#include "identification/foc_identification.h"
#include "sweep/foc_sweep.h"
#include "common.h"
#include "foc_modulation.h"
#include "board.h"
#include "foc_calibration.h"
#include "foc_sense.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "current_sense.h"
#include "foc_pid.h"
#include "motor.h"
#include "../config/motor_config.h"
#include "dwt_profile_delay.h"
#include "control_mode.h"
#include "math_compat.h"
#include <string.h>
#include <float.h>

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
  const float meas_fc = FOC_MEAS_LPF_FC_HZ;
  const float meas_fs = FOC_MEAS_LPF_FS_HZ;

  lpf1_init(&m->vbus_lpf, FOC_VBUS_LPF_FC_HZ, FOC_VBUS_SAMPLE_FS_HZ);

  lpf1_init(&m->v_a_lpf, meas_fc, meas_fs);
  lpf1_init(&m->v_b_lpf, meas_fc, meas_fs);
  lpf1_init(&m->v_c_lpf, meas_fc, meas_fs);
  lpf1_init(&m->i_d_lpf, meas_fc, meas_fs);
  lpf1_init(&m->i_q_lpf, meas_fc, meas_fs);
  lpf1_init(&m->i_mod_lpf, meas_fc, meas_fs);
  lpf1_init(&m->i_bus_lpf, meas_fc, meas_fs);
  lpf1_init(&m->v_d_lpf, meas_fc, meas_fs);
  lpf1_init(&m->v_q_lpf, meas_fc, meas_fs);
}

/* Both the closed-loop and explicit-angle callers supply the trig pair they need. */
#if defined(__CC_ARM)
__attribute__((noinline)) /* Keep the SRAM scatter entry even when there is only one caller. */
#endif
static void foc_apply_voltage(motor_handle_t *m, float uq, float ud, float sin_el, float cos_el)
{
  foc_modulation_t modulation;
  float duty_max = AT32_FOC_CURRENT_MAX_DUTY;
  float vbus = m->state.vbus_filtered;
  if (!(vbus > FLT_EPSILON))
  {
    vbus = m->config.limits.vbus_nominal;
    if (!(vbus > FLT_EPSILON)) vbus = BUS_VOLTAGE;
  }
  if (!foc_modulate_voltage_sincos(ud, uq, sin_el, cos_el, vbus, duty_max, &modulation))
  {
    fault_t fault = duty_max > FLT_EPSILON && duty_max <= 1.0f ? FAULT_PARAM : FAULT_CURRENT_SENSOR;
    motor_fault_raise(m, fault);
    motor_disarm(m);
    return;
  }

  m->inv_park_result.u_alpha = modulation.u_alpha;
  m->inv_park_result.u_beta = modulation.u_beta;
  m->inv_clarke_result.u_a = modulation.u_a;
  m->inv_clarke_result.u_b = modulation.u_b;
  m->inv_clarke_result.u_c = modulation.u_c;
  m->out.duty_a = modulation.duty_a;
  m->out.duty_b = modulation.duty_b;
  m->out.duty_c = modulation.duty_c;
  (void)motor_apply_pwm(m, &m->out, duty_max);
}

/* Open-loop and calibration commands own their angle, including at startup. */
void foc_voltage(motor_handle_t *m, float uq, float ud, float angle_el)
{
  float sin_el, cos_el;
  if (m == NULL) return;

  if (foc_modulation_finite(angle_el)) foc_sincos(angle_el, &sin_el, &cos_el);
  else sin_el = cos_el = angle_el; /* Reject before a nonfinite angle reaches libm. */
  foc_apply_voltage(m, uq, ud, sin_el, cos_el);
}

static void foc_current(motor_handle_t *m, motor_runtime_t *rt, float iq)
{
  uint32_t start_ticks = rt != NULL ? dwt_get_ticks() : 0u;
  float vd = 0.0f;
  float vq = 0.0f;
  const pid_param_t *id_cfg = &m->config.pid.id;
  const pid_param_t *iq_cfg = &m->config.pid.iq;

#if FOC_CUR_LOOP_FAST
  if (id_cfg->ki == 0.0f && id_cfg->kd == 0.0f && id_cfg->ramp == 0.0f)
  {
    vd = -id_cfg->kp * m->state.i_d;
    m->id_pid.timestamp += (uint32_t)(m->id_pid.dt * 1.0e6f + 0.5f);
    m->id_pid.target = 0.0f;
    m->id_pid.feedback = m->state.i_d;
    m->id_pid.last_error = -m->state.i_d;
    m->id_pid.last_output = vd;
  }
  else
  {
    (void)foc_pid_calc(&m->id_pid, id_cfg, 0.0f, m->state.i_d, &vd);
  }
  if (iq_cfg->kd == 0.0f && iq_cfg->ramp == 0.0f && iq_cfg->limit > FLT_EPSILON)
  {
    (void)foc_pid_calc_iq_current(&m->iq_pid, iq_cfg, iq, m->state.i_q, &vq);
  }
  else
  {
    (void)foc_pid_calc(&m->iq_pid, iq_cfg, iq, m->state.i_q, &vq);
  }
#else
  (void)foc_pid_calc(&m->id_pid, id_cfg, 0.0f, m->state.i_d, &vd);
  (void)foc_pid_calc(&m->iq_pid, iq_cfg, iq, m->state.i_q, &vq);
#endif

  m->state.v_d = vd;
  m->state.v_q = vq;

#if FOC_REUSE_PARK_SINCOS
  foc_apply_voltage(m, vq, vd, m->state.sin_elec, m->state.cos_elec);
#else
  foc_voltage(m, vq, vd, m->state.theta_elec_rad);
#endif

  if (rt != NULL) rt->foc_cur_loop_us = FOC_PROFILE_CST_OUTER ? dwt_elapsed_us(start_ticks, dwt_get_ticks()) : 0u;
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
    float spin_sign = rt->open_loop_omega;
    float gear_ratio = (m->config.param.gear_ratio > 0.0f) ? m->config.param.gear_ratio : 1.0f;
    float uq_mag = OPEN_LOOP_UQ_MAX;

    if (fabsf(spin_sign) < OPEN_LOOP_OMEGA_EPS_RAD_S)
    {
      spin_sign = target_speed;
    }
    if (spin_sign < 0.0f)
    {
      uq_mag = -uq_mag;
    }
    rt->open_loop_uq = uq_mag;
    /* ref.speed / open_loop_omega 为关节端 [rad/s]；电角速率 = pp × ω_shaft = pp × ω_joint × gear_ratio */
    rt->open_loop_elec_angle_rad = angle_normalize(
        rt->open_loop_elec_angle_rad
        + (float)m->config.param.pole_pairs * rt->open_loop_omega * gear_ratio * dt);
  }

  foc_voltage(m, rt->open_loop_uq, 0.0f, rt->open_loop_elec_angle_rad);
}

static void foc_update_speed_pid(motor_handle_t *m, float target_speed)
{
  motor_runtime_t *rt = motor_runtime(m);
  float iq_cmd = 0.0f;
  uint32_t start_ticks;

  if (m == NULL)
  {
    return;
  }

  start_ticks = dwt_get_ticks();
  if (foc_pid_calc(&m->velocity_pid,
                   &m->config.pid.velocity,
                   target_speed,
                   m->state.joint.velocity_rad_s,
                   &iq_cmd) != ERR_NONE)
  {
    m->velocity_pid.last_output = 0.0f;
  }
  if (rt != NULL)
  {
    rt->foc_vel_loop_us = dwt_elapsed_us(start_ticks, dwt_get_ticks());
  }
}

static float foc_position_speed_ref(motor_handle_t *m, float target_pos)
{
  motor_runtime_t *rt = motor_runtime(m);
  float speed_cmd = 0.0f;
  uint32_t start_ticks;
  if (m == NULL)
  {
    return 0.0f;
  }
  start_ticks = dwt_get_ticks();
  (void)foc_pid_calc(&m->position_pid,
                     &m->config.pid.position,
                     target_pos,
                     m->state.joint.position_rad,
                     &speed_cmd);
  if (rt != NULL)
  {
    rt->foc_pos_loop_us = dwt_elapsed_us(start_ticks, dwt_get_ticks());
  }
  if (m->config.motion.max_speed_rad_s > 0.0f)
  {
    speed_cmd = CLAMP(speed_cmd,
                      -m->config.motion.max_speed_rad_s,
                      m->config.motion.max_speed_rad_s);
  }

  return speed_cmd;
}

static void foc_reject_control_mode(motor_handle_t *m)
{
  motor_disarm(m);
  if (m->ctrl_mode == CTRL_MODE_IDLE)
  {
    foc_fsm_next_state(m, STATE_IDLE);
  }
  else
  {
    motor_fault_raise(m, FAULT_PARAM);
    foc_fsm_next_state(m, STATE_FAULT);
  }
}

static void foc_control_run(motor_handle_t *m, motor_runtime_t *rt, bool outer_loop_skip)
{

  if (rt != NULL)
  {
    rt->current_iq_limited = 0.0f;
    rt->current_speed_limited = false;
    rt->foc_pos_loop_us = 0u;
    rt->foc_vel_loop_us = 0u;
    rt->foc_cur_loop_us = 0u;
  }

  switch (m->ctrl_mode)
  {
    /* PT-compatible torque and cyclic torque share protection and current PID.
     * CST consumes the latest host Iq each ISR; no host-rate interpolation. */
    case CTRL_MODE_CST:
    case CTRL_MODE_TORQUE:
    {
      float iq_limit = m->config.limits.iq_limit;
      float iq_ref = m->ref.iq;

      if (iq_limit > 0.0f)
      {
        iq_ref = CLAMP(iq_ref, -iq_limit, iq_limit);
      }
      if (rt != NULL) {
        /** 扭矩模式下，电机轴速度限制值. 避免电机轴速度超过限制值，导致电机轴过热。 */
        float limited = speed_limit_soft_iq(iq_ref, m->state.shaft.velocity_rad_s,
                                            rt->current_soft_speed_rad_s);
        rt->current_speed_limited = (limited != iq_ref);
        iq_ref = limited;
        rt->current_iq_limited = iq_ref;
      }
      foc_current(m, rt, iq_ref);
      break;
    }
    case CTRL_MODE_SPEED:
    case CTRL_MODE_POSITION:
      if (outer_loop_skip)
      {
        float target_speed = m->ctrl_mode == CTRL_MODE_POSITION
                           ? foc_position_speed_ref(m, m->ref.position_rad)
                           : m->ref.speed_rad_s;
        foc_update_speed_pid(m, target_speed);
      }
      foc_current(m, rt, m->velocity_pid.last_output);
      break;
    case CTRL_MODE_SPEED_OPEN_LOOP:
      foc_speed_open_control(m, m->ref.speed_rad_s);
      break;
    default:
      foc_reject_control_mode(m);
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
  rt->current_soft_speed_rad_s = 0.9f * MAX_VELOCITY_RAD_S;
  rt->shaft_hard_overspeed_enabled = true;
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

  (void)foc_pid_init(&m->id_pid, FOC_PID_LOOP_CURRENT);
  (void)foc_pid_init(&m->iq_pid, FOC_PID_LOOP_CURRENT);
  (void)foc_pid_init(&m->velocity_pid, FOC_PID_LOOP_VELOCITY);
  (void)foc_pid_init(&m->position_pid, FOC_PID_LOOP_POSITION);
  foc_init_filters(m);

  if (m->config.limits.vbus_nominal > 0.0f)
  {
    m->state.vbus = m->config.limits.vbus_nominal;
    m->state.vbus_filtered = m->config.limits.vbus_nominal;
    lpf1_reset(&m->vbus_lpf, m->state.vbus);
  }

  current_sense_init(m);
  if (!m->current_offset_cal_done)
  {
    return; /* Never attempt electrical alignment after failed current sensing. */
  }

  if (m->config.auto_align_electrical)
  {
    cal_elec_align(m);
  }
}

#if defined(__ARMCC_VERSION)
__weak void foc_control_loop(motor_handle_t *m)
#elif defined(__GNUC__)
__attribute__((weak)) void foc_control_loop(motor_handle_t *m)
#else
void foc_control_loop(motor_handle_t *m)
#endif
{
  uint32_t loop_start_ticks = dwt_get_ticks();
  uint32_t sense_start_ticks = 0u, sense_end_ticks = 0u;
  foc_sense_marks_t marks;

  if (m == NULL)
  {
    return;
  }
  motor_runtime_t *rt = motor_runtime(m);

  bool running_cst = rt != NULL && m->fsm == STATE_RUNNING && m->ctrl_mode == CTRL_MODE_CST;
  bool profile_cst = FOC_PROFILE_CST_OUTER == 1 && running_cst;
  uint32_t ctrl_ticks = 0u;

  /* Reject before sensor I/O/FSM can arm hardware or retain an old vector. */
  if (m->fsm == STATE_RUNNING && !control_mode_needs_pwm(m->ctrl_mode))
  {
    foc_reject_control_mode(m);
    return;
  }

  /**
   * All outer work uses the same phase. Start immediately when entering RUNNING. 
   * 外环间隔：FW_FOC_OUTER_LOOP_DIV，outer_loop_skip == 0u 表示外环拍
   */
  bool outer_loop_skip = rt == NULL || rt->outer_loop_skip == 0u ||
                    (m->fsm == STATE_RUNNING && rt->fsm_enter_pending);
  if (rt != NULL)
  {
    rt->outer_loop_skip = outer_loop_skip ? FW_FOC_OUTER_LOOP_DIV - 1u : rt->outer_loop_skip - 1u;
  }

  if (profile_cst)
  {
    rt->cst_shaft_spi_ticks = 0u;
    rt->profile_cst_outer = true;
    sense_start_ticks = dwt_get_ticks();
  }

  uint32_t sense_ticks = foc_sense_update_timed(m, profile_cst ? &marks : NULL, outer_loop_skip);
  if (profile_cst) sense_end_ticks = dwt_get_ticks();

  /* 电机轴速度保护(非关节轴速度保护) */
  if (motor_fault_check_shaft_speed(m)) {
    foc_on_fault_latched(m);
    if (running_cst)
    {
      rt->foc_ctrl_loop_us = rt->foc_cur_loop_us = 0u;
      goto publish_timing;
    }
    return;
  }

  foc_fsm_loop(m);
  if (foc_ident_active(m)) foc_ident_run_isr(m);
  if (m->fsm == STATE_RUNNING)
  {
    uint32_t ctrl_start_ticks = profile_cst ? dwt_get_ticks() : 0u;
    /* Single optional dispatch gate; normal PID/modulation contain no test hooks. */
    if (foc_sweep_is_active(m)) foc_sweep_run_isr(m, rt, loop_start_ticks, foc_current);
    else foc_control_run(m, rt, outer_loop_skip);
    if (rt != NULL && profile_cst)
    {
      ctrl_ticks = dwt_get_ticks() - ctrl_start_ticks;
      rt->foc_ctrl_loop_us = dwt_ticks_to_us(ctrl_ticks);
    }
  }
  else if (running_cst)
  {
    rt->foc_ctrl_loop_us = rt->foc_cur_loop_us = 0u;
  }

publish_timing:
  if (rt != NULL)
  {
    uint32_t loop_end_ticks = dwt_get_ticks();
    uint32_t loop_ticks = loop_end_ticks - loop_start_ticks;
    rt->foc_control_loop_us = dwt_ticks_to_us(loop_ticks);
    if (profile_cst)
    {
      /* Subtract the shaft SPI interval before converting the position stage to us. */
      uint32_t position_ticks = marks.position_end - marks.current_end; // 即 encoder_update(m, outer_tick) 耗时
      rt->foc_sense_update_us = dwt_elapsed_us(sense_start_ticks, sense_end_ticks);
      rt->foc_current_sense_us = dwt_elapsed_us(sense_start_ticks, marks.current_end);
      rt->foc_encoder_update_us = dwt_ticks_to_us(rt->cst_shaft_spi_ticks);
      rt->foc_pos_loop_us = dwt_ticks_to_us(position_ticks - rt->cst_shaft_spi_ticks); // 位置更新耗时, 即 encoder_update(m, outer_tick) 耗时减去电机轴 SPI 耗时
      rt->foc_vel_loop_us = dwt_elapsed_us(marks.position_end, marks.encoder_end); // 速度更新耗时(根据位置差算速度)
      rt->profile_cst_outer = false;
    }
    else if (running_cst)
    {
      /* Reuse the idle CST outer PID channels in the same debug sample. */
      rt->foc_pos_loop_us = FOC_PROFILE_CST_OUTER ? 1u : 0u;
      rt->foc_vel_loop_us = FOC_PROFILE_CST_OUTER ? dwt_ticks_to_us(loop_ticks - sense_ticks - ctrl_ticks) : 0u;
    }
  }
}
