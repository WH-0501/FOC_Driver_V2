#include "foc.h"
#include "fra.h"
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
#include "common.h"
#include "control_mode.h"
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

static void foc_store_control_loop_us(motor_runtime_t *rt, uint32_t t0_us)
{
  if (rt != NULL)
  {
    rt->foc_control_loop_us = foc_dwt_elapsed_us(t0_us);
  }
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

void foc_voltage(motor_handle_t *m, float Uq, float Ud, float angle_el)
{
  foc_modulation_t modulation;
  float vbus;
  float duty_max;

  if (m == NULL)
  {
    return;
  }

  duty_max = pwm_hw_max_duty();
  if (!(duty_max > 0.0f && duty_max <= 1.0f))
  {
    motor_fault_raise(m, FAULT_CURRENT_SENSOR);
    motor_disarm(m);
    return;
  }

  vbus = m->state.vbus_filtered;
  if (vbus <= 0.0f)
  {
    vbus = m->config.limits.vbus_nominal;
  }
  if (vbus <= 0.0f)
  {
    vbus = BUS_VOLTAGE;
  }

  if (!foc_modulate_voltage(Ud, Uq, angle_el, vbus, duty_max, &modulation))
  {
    motor_fault_raise(m, FAULT_PARAM);
    motor_disarm(m);
    return;
  }

  /* 调制器缩小了电压矢量，说明母线电压已经饱和，扫频结果不再线性。 */
  if (fra_owned() && (modulation.voltage_scale < 0.9999f))
  {
    fra_abort(FRA_FLAG_LIMIT);
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
  (void)motor_apply_pwm(m, &m->out);
}

static void foc_current(motor_handle_t *m, float id, float iq, float angle_el, float phase_vel)
{
  motor_runtime_t *rt = motor_runtime(m);
  float v_d_ctrl = 0.0f;
  float v_q_ctrl = 0.0f;
  /* 本拍 PID 之前的 q 轴电压，用来判断斜率限制有没有改写输出。 */
  float vq_before_pid = m->iq_pid.last_output;
  /* 本拍入口处扫频是否仍在采集。分段耗时只在采集期间记录。 */
  bool fra_acquiring = (fra_state() == FRA_STATE_ACQUIRING);
  /* 电流 PI 分段的起始时间戳，单位 us，按 16 位回绕相减。 */
  uint16_t stage_t0_us = fra_acquiring ? (uint16_t)board_time_us() : 0u;
  uint32_t t0_us = dwt_get_ticks_us();

  (void)phase_vel;

  (void)foc_pid_calc(&m->id_pid, &m->config.pid.id, id, m->state.i_d, &v_d_ctrl);
  (void)foc_pid_calc(&m->iq_pid, &m->config.pid.iq, iq, m->state.i_q, &v_q_ctrl);

  if (fra_acquiring)
  {
    fra_stage(FRA_STAGE_CURRENT_PID,
              (uint16_t)((uint16_t)board_time_us() - stage_t0_us));
  }
  /* 本拍 Vq 变化达到斜率上限的 99.9%，斜坡限制正在起作用。 */
  if (fra_owned() && (m->config.pid.iq.ramp > 0.0f) && (m->iq_pid.dt > 0.0f) &&
      (fabsf(v_q_ctrl - vq_before_pid) >= m->config.pid.iq.ramp * m->iq_pid.dt * 0.999f))
  {
    fra_abort(FRA_FLAG_LIMIT);
  }

  m->state.v_d = v_d_ctrl;
  m->state.v_q = v_q_ctrl;
  if (fra_acquiring)
  {
    stage_t0_us = (uint16_t)board_time_us();
  }
  foc_voltage(m, v_q_ctrl, v_d_ctrl, angle_el);
  if (fra_acquiring)
  {
    fra_stage(FRA_STAGE_VOLTAGE,
              (uint16_t)((uint16_t)board_time_us() - stage_t0_us));
  }

  m->state.i_bus = sqrtf(SQ(m->state.i_d) + SQ(m->state.i_q));

  if (rt != NULL)
  {
    rt->foc_cur_loop_us = foc_dwt_elapsed_us(t0_us);
  }
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
    rt->current_iq_limited = 0.0f;
    rt->current_speed_limited = false;
    rt->foc_pos_loop_us = 0.0f;
    rt->foc_vel_loop_us = 0.0f;
    rt->foc_cur_loop_us = 0.0f;
  }

  switch (m->ctrl_mode)
  {
    /* PT-compatible torque and cyclic torque share protection and current PID.
     * CST consumes the latest host Iq each ISR; no host-rate interpolation. */
    case CTRL_MODE_CST:
    case CTRL_MODE_TORQUE:
    {
      float iq_limit = m->config.limits.iq_limit;
      float iq_ref;

      if (fra_owned())
      {
        /* 扫频占用 Iq：用本拍激励替换主机电流指令，并记下注入耗时。 */
        uint16_t inject_t0_us = (uint16_t)board_time_us();

        m->ref.iq = fra_step(m->state.i_q);
        fra_stage(FRA_STAGE_INJECT,
                  (uint16_t)((uint16_t)board_time_us() - inject_t0_us));
      }
      iq_ref = m->ref.iq;

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
      /* 电流上限或速度软限位改写了激励幅值。 */
      if (fra_owned() && (iq_ref != m->ref.iq))
      {
        fra_abort(FRA_FLAG_LIMIT);
        iq_ref = 0.0f;
        m->ref.iq = 0.0f;
      }
      foc_current(m, 0.0f, iq_ref, m->state.theta_elec_rad, 0.0f);
      /* q 轴电压指令顶到 PID 输出上限。 */
      if (fra_owned() && (fabsf(m->iq_pid.last_output) >= m->config.pid.iq.limit * 0.999f))
      {
        fra_abort(FRA_FLAG_LIMIT);
      }
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
      foc_reject_control_mode(m);
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
  /* 扫频前最近一次正常 CST 周期的入口、耗时和间隔，用作对比基线。 */
  static uint16_t s_cst_prev_entry_us;
  static uint16_t s_cst_duration_us;
  static uint16_t s_cst_interval_us;
  static bool s_cst_baseline_valid;
  /* 上一拍扫频采集的入口时间，用来测量电流环间隔。 */
  static uint16_t s_fra_prev_entry_us;
  /* 本拍入口处是否正在采集。中途状态变化也按这一拍的采样结果记时。 */
  bool fra_acquiring = (fra_state() == FRA_STATE_ACQUIRING);
  bool sample_cst_baseline = !fra_acquiring && (m != NULL) &&
                              (m->fsm == STATE_RUNNING) &&
                              (m->ctrl_mode == CTRL_MODE_CST);
  uint16_t cst_entry_us = 0u;
  uint16_t fra_entry_us = 0u;
  uint16_t stage_t0_us = 0u;
  uint32_t fra_cycle_ticks = 0u;
  uint32_t t_loop_us;
  motor_runtime_t *rt = motor_runtime(m);

  if (sample_cst_baseline)
  {
    cst_entry_us = (uint16_t)board_time_us();
    s_cst_interval_us = s_cst_baseline_valid
                            ? (uint16_t)(cst_entry_us - s_cst_prev_entry_us)
                            : 0u;
    s_cst_prev_entry_us = cst_entry_us;
  }
  else
  {
    /* 采集第一拍还没有样本时，提交扫频前那次 CST 的耗时和间隔。 */
    if (fra_acquiring && (fra_samples() == 0u) && s_cst_baseline_valid)
    {
      fra_baseline(s_cst_duration_us, s_cst_interval_us);
    }
    s_cst_baseline_valid = false;
  }
  if (fra_acquiring)
  {
    uint16_t interval_us;

    fra_cycle_ticks = dwt_get_ticks();
    fra_entry_us = (uint16_t)board_time_us();
    interval_us = (uint16_t)(fra_entry_us - s_fra_prev_entry_us);
    /* 第一拍没有上一拍入口，从已有样本的下一拍开始检查间隔。 */
    if (fra_samples() > 0u)
    {
      fra_interval(interval_us);
    }
    s_fra_prev_entry_us = fra_entry_us;
  }
  if (m == NULL)
  {
    return;
  }

  t_loop_us = dwt_get_ticks_us();

  /* Reject before sensor I/O/FSM can arm hardware or retain an old vector. */
  if (m->fsm == STATE_RUNNING && !control_mode_needs_pwm(m->ctrl_mode))
  {
    foc_reject_control_mode(m);
    foc_store_control_loop_us(rt, t_loop_us);
    return;
  }
  // foc_sense_poll_vbus(m);
  if (fra_acquiring)
  {
    stage_t0_us = (uint16_t)board_time_us();
  }
  foc_sense_update(m);
  if (fra_acquiring)
  {
    fra_stage(FRA_STAGE_SENSE,
              (uint16_t)((uint16_t)board_time_us() - stage_t0_us));
  }
  
  /* Independent of mode, user speed commands, runtime config and gear ratio. */
  if (motor_fault_check_shaft_speed(m)) {
    foc_on_fault_latched(m);
    foc_store_control_loop_us(rt, t_loop_us);
    return;
  }
  foc_fsm_loop(m);
  if (m->fsm == STATE_RUNNING)
  {
    if (fra_acquiring)
    {
      stage_t0_us = (uint16_t)board_time_us();
    }
    foc_control_run(m);
    if (fra_acquiring)
    {
      fra_stage(FRA_STAGE_CONTROL,
                (uint16_t)((uint16_t)board_time_us() - stage_t0_us));
    }
  }
  if (sample_cst_baseline)
  {
    s_cst_duration_us = (uint16_t)((uint16_t)board_time_us() - cst_entry_us);
    s_cst_baseline_valid = true;
  }
  if (fra_acquiring)
  {
    uint16_t loop_us = (uint16_t)((uint16_t)board_time_us() - fra_entry_us);

    /* 先按本拍耗时刷新周期计数峰值，fra_timing 随后才更新最大耗时。 */
    fra_cycle_measure(dwt_get_ticks() - fra_cycle_ticks, loop_us);
    fra_timing(loop_us);
  }
  /* 采集已结束：清掉激励、卸力，并把扫频占用交还。 */
  if (fra_owned() && (fra_state() != FRA_STATE_ACQUIRING))
  {
    m->ref.iq = 0.0f;
    motor_disarm(m);
    foc_fsm_next_state(m, STATE_IDLE);
    fra_release();
  }
  foc_store_control_loop_us(rt, t_loop_us);
}
