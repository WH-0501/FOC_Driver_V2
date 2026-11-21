#include "foc_sweep.h"
#if FOC_SWEEP_ENABLED
#include "board.h"
#include "common.h"
#include "dwt_profile_delay.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include <float.h>
#include <math.h>
#include <string.h>

static sweep_engine_t engine;
motor_handle_t * volatile foc_sweep_owner;
#define owner foc_sweep_owner
static uint16_t run_id;
static uint32_t previous_ticks, max_us;
static bool have_previous;
static float origin, peak;
static struct {
  uint16_t interval_min_us, interval_max_us, interval_last_us;
  uint16_t sense_max_us, control_max_us, engine_max_us;
  /* Aborted engine.samples already retains the failure sample count. Reuse
   * that duplicate storage and structure padding: still 20 bytes total. */
  float motion_value;
  uint8_t timing_reason, motion_reason;
} timing;
static uint16_t bounded_us(uint32_t us) { return (uint16_t)(us > 65535u ? 65535u : us); }

static bool finite_value(float f) { return f >= -FLT_MAX && f <= FLT_MAX; }
/* Keep ARM ISR layout stable when foreground diagnostic callers change. */
#if defined(__CC_ARM)
__attribute__((noinline))
#endif
static bool allowed(const motor_handle_t *m)
{
  return m && m == motor_get_handler() && m->fsm == STATE_RUNNING &&
      m->ctrl_mode == CTRL_MODE_CST && motor_is_armed(m) && m->out.pwm_enable &&
      m->current_offset_cal_done && !m->fault.latched && !m->fault.status &&
      m->ref.iq == 0.0f;
}
static unsigned measurement_guard(const motor_handle_t *m)
{
  float position = m->state.shaft.position_rad, speed = m->state.shaft.velocity_rad_s;
  if (!finite_value(position) || !finite_value(speed) ||
      !finite_value(m->state.i_q) || !finite_value(m->state.i_d) ||
      !finite_value(m->state.board_temp.value) || !finite_value(m->state.motor_temp.value))
    return SWEEP_NONFINITE;
  if (fabsf(position - origin) >= FOC_SWEEP_MAX_TRAVEL_RAD ||
      fabsf(speed) >= FOC_SWEEP_MAX_SPEED_RAD_S) return SWEEP_MOTION;
  if (!(m->config.limits.iq_limit > 0.0f) || !finite_value(m->config.limits.iq_limit) ||
      peak > m->config.limits.iq_limit * 0.8f ||
      m->state.board_temp.value >= FOC_SWEEP_MAX_TEMP_C ||
      m->state.motor_temp.value >= FOC_SWEEP_MAX_TEMP_C) return SWEEP_LIMIT;
  return 0u;
}
/* Called with IRQs masked from foreground, or from the control ISR. owner is
 * cleared BEFORE motor_disarm to avoid recursive callbacks and to preserve DONE. */
static void finish(motor_handle_t *m, unsigned flags)
{
  if (owner != m || !m) return;
  if (flags) sweep_engine_abort(&engine, flags);
  owner = NULL;
  m->ref.iq = 0.0f;
  if (m->fsm != STATE_FAULT && !m->fault.latched) foc_fsm_next_state(m, STATE_IDLE);
  motor_disarm(m);
}
/* Cold abort path only: snapshot BEFORE disarm; no work added to the normal
 * measurement guard. Travel has precedence when both bounds are exceeded. */
#if defined(__CC_ARM)
__attribute__((noinline))
#endif
static void finish_guard(motor_handle_t *m, unsigned flags)
{
  if (flags == SWEEP_MOTION) {
    float travel = m->state.shaft.position_rad - origin;
    if (fabsf(travel) >= FOC_SWEEP_MAX_TRAVEL_RAD) {
      timing.motion_reason = SWEEP_REJECT_TRAVEL;
      timing.motion_value = travel;
    } else {
      timing.motion_reason = SWEEP_REJECT_SPEED;
      timing.motion_value = m->state.shaft.velocity_rad_s;
    }
  }
  finish(m, flags);
}
static int reject_start(foc_sweep_rejection_t *out, int code, unsigned reason, float value)
{
  if (out) { out->reason = (uint8_t)reason; out->value = value; }
  return code;
}
/* Kept outside the ISR: identify the condition, do not relax allowed(). */
static int start_admission(const motor_handle_t *m, foc_sweep_rejection_t *out)
{
  if (allowed(m)) return 0; /* share the exact normal admission predicate */
  if (!m || m != motor_get_handler()) return reject_start(out, 1, SWEEP_REJECT_MOTOR, 0);
  if (m->fsm != STATE_RUNNING) return reject_start(out, 1, SWEEP_REJECT_STATE, (float)m->fsm);
  if (m->ctrl_mode != CTRL_MODE_CST) return reject_start(out, 1, SWEEP_REJECT_MODE, (float)m->ctrl_mode);
  if (!motor_is_armed(m) || !m->out.pwm_enable) return reject_start(out, 1, SWEEP_REJECT_DISARMED, 0);
  if (!m->current_offset_cal_done) return reject_start(out, 1, SWEEP_REJECT_CALIBRATION, 0);
  if (m->fault.latched || m->fault.status) return reject_start(out, 1, SWEEP_REJECT_FAULT, (float)(m->fault.latched | m->fault.status));
  if (m->ref.iq != 0.0f) return reject_start(out, 1, SWEEP_REJECT_REFERENCE, m->ref.iq);
  return 0;
}
static int start_guard(const motor_handle_t *m, foc_sweep_rejection_t *out)
{
  unsigned flags = measurement_guard(m);
  if (!flags) return 0;
  if (flags == SWEEP_NONFINITE) return reject_start(out, 1, SWEEP_REJECT_NONFINITE, 0);
  if (flags == SWEEP_MOTION) {
    float travel = m->state.shaft.position_rad - origin;
    if (fabsf(travel) >= FOC_SWEEP_MAX_TRAVEL_RAD) return reject_start(out, 1, SWEEP_REJECT_TRAVEL, travel);
    return reject_start(out, 1, SWEEP_REJECT_SPEED, m->state.shaft.velocity_rad_s);
  }
  if (!(m->config.limits.iq_limit > 0.0f) || !finite_value(m->config.limits.iq_limit) || peak > m->config.limits.iq_limit * 0.8f)
    return reject_start(out, 1, SWEEP_REJECT_CURRENT_LIMIT, m->config.limits.iq_limit);
  if (m->state.board_temp.value >= FOC_SWEEP_MAX_TEMP_C) return reject_start(out, 1, SWEEP_REJECT_BOARD_TEMP, m->state.board_temp.value);
  return reject_start(out, 1, SWEEP_REJECT_MOTOR_TEMP, m->state.motor_temp.value);
}
int foc_sweep_start(motor_handle_t *m, const sweep_config_t *cfg, uint16_t run)
{
  return foc_sweep_start_checked(m, cfg, run, NULL);
}
int foc_sweep_start_checked(motor_handle_t *m, const sweep_config_t *cfg, uint16_t run,
                            foc_sweep_rejection_t *rejection)
{
  uint32_t irq;
  int result;
  /* Single foreground caller. Preparation cannot be published to ISR early. */
  if (rejection) { rejection->reason = SWEEP_REJECT_NONE; rejection->value = 0; }
  if (owner) return reject_start(rejection, 1, SWEEP_REJECT_BUSY, 0);
  result = start_admission(m, rejection);
  if (result) return result;
  if (!cfg || !run || cfg->sample_hz != (float)FOC_CURRENT_LOOP_FS_HZ)
    return reject_start(rejection, 2, SWEEP_REJECT_REQUEST, 0);
  if (!(m->config.limits.iq_limit > 0.0f) || !finite_value(m->config.limits.iq_limit) || cfg->peak > m->config.limits.iq_limit * 0.8f)
    return reject_start(rejection, 2, SWEEP_REJECT_CURRENT_LIMIT, m->config.limits.iq_limit);
  if (!finite_value(m->state.shaft.position_rad) || !finite_value(m->state.shaft.velocity_rad_s))
    return reject_start(rejection, 2, SWEEP_REJECT_NONFINITE, 0);
  if (fabsf(m->state.shaft.velocity_rad_s) > 1.0f) return reject_start(rejection, 2, SWEEP_REJECT_START_SPEED, m->state.shaft.velocity_rad_s);
  if (!(m->config.pid.iq.limit > 0.0f) || !finite_value(m->config.pid.iq.limit)) return reject_start(rejection, 2, SWEEP_REJECT_Q_LIMIT, m->config.pid.iq.limit);
  if (!(m->config.pid.id.limit > 0.0f) || !finite_value(m->config.pid.id.limit)) return reject_start(rejection, 2, SWEEP_REJECT_D_LIMIT, m->config.pid.id.limit);
  if (m->config.pid.iq.ramp != 0.0f) return reject_start(rejection, 2, SWEEP_REJECT_Q_RAMP, m->config.pid.iq.ramp);
  if (m->config.pid.id.ramp != 0.0f) return reject_start(rejection, 2, SWEEP_REJECT_D_RAMP, m->config.pid.id.ramp);
  result = sweep_engine_prepare(&engine, cfg);
  if (result) return reject_start(rejection, result, SWEEP_REJECT_FREQUENCIES, 0);
  irq = __get_PRIMASK();
  __disable_irq();
  /* Recheck after interruptible sin/cos/peak calculation. No libm under mask. */
  origin = m->state.shaft.position_rad;
  peak = cfg->peak;
  result = start_admission(m, rejection);
  if (!result) result = start_guard(m, rejection);
  if (!result) {
    run_id = run; max_us = 0u; have_previous = false;
    memset(&timing, 0, sizeof(timing));
    sweep_engine_start(&engine);
    __DMB();
    owner = m;
  }
  __set_PRIMASK(irq);
  return result;
}
void foc_sweep_stop(motor_handle_t *m, unsigned reason)
{
  uint32_t irq;
  if (!m || owner != m) return; /* inactive: no IRQ masking */
  irq = __get_PRIMASK();
  __disable_irq();
  finish(m, reason ? reason : SWEEP_STOP);
  __set_PRIMASK(irq);
}
void foc_sweep_on_disarm(motor_handle_t *m)
{
  uint32_t irq;
  if (!m || owner != m) return; /* inactive: no IRQ masking */
  irq = __get_PRIMASK();
  __disable_irq();
  if (owner == m && m) {
    sweep_engine_abort(&engine, SWEEP_STOP);
    owner = NULL;
    m->ref.iq = 0.0f;
    /* Prevent STATE_RUNNING's automatic hardware re-arm on the next ISR. */
    if (m->fsm != STATE_FAULT && !m->fault.latched) foc_fsm_next_state(m, STATE_IDLE);
  }
  __set_PRIMASK(irq);
}
static bool pid_limited(const pid_state_t *pid, const pid_param_t *cfg, float voltage)
{
  return !finite_value(voltage) || !(cfg->limit > 0.0f) ||
      fabsf(voltage) >= cfg->limit || cfg->ramp != 0.0f ||
      (cfg->ki != 0.0f && cfg->i_limit > 0.0f && fabsf(pid->last_integral) >= cfg->i_limit);
}
static bool voltage_limited(const motor_handle_t *m)
{
  /* Observe the original current loop AFTER it applies its normal protections.
   * Same circular limit as foc_modulation.h; no duplicate modulation or sin/cos.
   * A limited final sample is rejected before results can be published. */
  float bus = m->state.vbus_filtered;
  float vd = m->state.v_d, vq = m->state.v_q, limit;
  if (!(bus > FLT_EPSILON)) {
    bus = m->config.limits.vbus_nominal;
    if (!(bus > FLT_EPSILON)) bus = BUS_VOLTAGE;
  }
  limit = bus * (AT32_FOC_CURRENT_MAX_DUTY * ONE_DIV_SQRT_3);
  return !finite_value(bus) || !finite_value(limit) ||
      vd * vd + vq * vq > limit * limit ||
      pid_limited(&m->iq_pid, &m->config.pid.iq, vq) ||
      pid_limited(&m->id_pid, &m->config.pid.id, vd);
}
void foc_sweep_run_isr(motor_handle_t *m, motor_runtime_t *rt, uint32_t start_ticks,
                       foc_sweep_current_fn current_step)
{
  uint32_t control_ticks, engine_ticks, us, end_ticks;
  float reference, limited;
  unsigned guard;
  if (!m || owner != m) return;
  if (!rt || !current_step || !allowed(m)) { finish(m, SWEEP_STOP); return; }
  control_ticks = dwt_get_ticks();
  if (have_previous) {
    uint32_t interval = dwt_elapsed_us(previous_ticks, start_ticks);
    float period = 1000000.0f / (float)FOC_CURRENT_LOOP_FS_HZ;
    timing.interval_last_us = bounded_us(interval);
    if (!timing.interval_min_us || interval < timing.interval_min_us) timing.interval_min_us = bounded_us(interval);
    if (interval > timing.interval_max_us) timing.interval_max_us = bounded_us(interval);
    if (interval < period * 0.8f || interval > period * 1.2f) {
      timing.timing_reason = 1;
      finish(m, SWEEP_TIMING); return;
    }
  }
  previous_ticks = start_ticks; have_previous = true;
  guard = measurement_guard(m);
  if (guard) { finish_guard(m, guard); return; }
  engine_ticks = dwt_get_ticks();
  reference = sweep_engine_step(&engine, m->state.i_q);
  us = dwt_elapsed_us(engine_ticks, dwt_get_ticks());
  if (us > timing.engine_max_us) timing.engine_max_us = bounded_us(us);
  if (engine.state == SWEEP_ABORTED) { finish(m, engine.flags); return; }
  limited = CLAMP(reference, -m->config.limits.iq_limit, m->config.limits.iq_limit);
  limited = speed_limit_soft_iq(limited, m->state.shaft.velocity_rad_s, rt->current_soft_speed_rad_s);
  rt->current_iq_limited = limited;
  rt->current_speed_limited = limited != reference;
  rt->foc_pos_loop_us = rt->foc_vel_loop_us = rt->foc_cur_loop_us = 0u;
  if (limited != reference) { finish(m, SWEEP_LIMIT); return; }
  /* The original current PID/modulation path is shared without any sweep hooks. */
  current_step(m, rt, reference);
  if (owner != m) return; /* motor_disarm/shutdown may have cancelled us */
  if (!allowed(m)) { finish(m, SWEEP_STOP); return; }
  if (voltage_limited(m)) { finish(m, SWEEP_LIMIT); return; }
  end_ticks = dwt_get_ticks();
  us = dwt_elapsed_us(control_ticks, end_ticks);
  if (us > timing.control_max_us) timing.control_max_us = bounded_us(us);
  if (rt->foc_sense_update_us > timing.sense_max_us) timing.sense_max_us = bounded_us(rt->foc_sense_update_us);
  us = dwt_elapsed_us(start_ticks, end_ticks);
  if (us > max_us) max_us = us;
  if (us > (1000000.0f / (float)FOC_CURRENT_LOOP_FS_HZ) * 0.9f) {
    timing.timing_reason = 2;
    finish(m, SWEEP_TIMING);
  } else if (engine.state == SWEEP_DONE) finish(m, 0u);
}
void foc_sweep_status(foc_sweep_status_t *out)
{
  uint32_t irq;
  if (!out) return;
  irq = __get_PRIMASK(); __disable_irq();
  out->run = run_id; out->flags = (uint16_t)engine.flags;
  out->max_us = (uint16_t)(max_us > 65535u ? 65535u : max_us);
  out->state = (uint8_t)engine.state; out->method = (uint8_t)engine.method;
  out->count = (uint8_t)engine.count; out->progress = (uint8_t)engine.progress;
  out->sample_hz = (uint32_t)engine.sample_hz; out->samples = engine.samples;
  out->interval_min_us = timing.interval_min_us; out->interval_max_us = timing.interval_max_us;
  out->interval_last_us = timing.interval_last_us; out->timing_reason = timing.timing_reason;
  out->failure_sample = engine.state == SWEEP_ABORTED ? engine.samples : 0u;
  out->sense_max_us = timing.sense_max_us;
  out->control_max_us = timing.control_max_us; out->engine_max_us = timing.engine_max_us;
  out->motion_reason = engine.state == SWEEP_ABORTED && (engine.flags & SWEEP_MOTION)
      ? timing.motion_reason : 0u;
  out->motion_value = out->motion_reason ? timing.motion_value : 0.0f;
  out->motion_index = out->motion_reason && engine.method == SWEEP_STEPPED_SINE
      ? engine.index : 255u;
  __set_PRIMASK(irq);
  /* No division under IRQ mask. A stopped engine is immutable until the next
   * foreground start. Multisine has simultaneous tones, not one current Hz. */
  out->motion_limit = out->motion_reason == SWEEP_REJECT_TRAVEL ? FOC_SWEEP_MAX_TRAVEL_RAD
      : out->motion_reason == SWEEP_REJECT_SPEED ? FOC_SWEEP_MAX_SPEED_RAD_S : 0.0f;
  out->motion_hz = out->motion_index < out->count
      ? sweep_engine_frequency(&engine, out->motion_index) : 0.0f;
}
int foc_sweep_result(uint16_t run, unsigned index, sweep_result_t *out)
{
  /* Only foreground starts a new sweep. Completed buffers are immutable in ISR
   * after owner is cleared, so sqrt/division do not need an IRQ mask. */
  if (owner || run != run_id) return 1;
  __DMB();
  return sweep_engine_result(&engine, index, out);
}
#endif
