#if defined(__CC_ARM)
#pragma Ospace
#endif
#include "foc_identification.h"
#include "motor.h"
#include "mc_interface.h"
#include "foc.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "board.h"
#include "common.h"
#include "dwt_profile_delay.h"
#include <float.h>
#include <math.h>
#include <string.h>
static ident_engine_t engine;
static motor_handle_t * volatile owner;
static motor_param_t original;
static uint32_t previous_ticks;
static bool have_previous;
static bool finite_f(float x) { return x >= -FLT_MAX && x <= FLT_MAX; }

void foc_ident_defaults(motor_handle_t *m, ident_config_t *c) {
  static const ident_config_t defaults = {0.3f, 3.0f, 0.2f, 200.0f, 20.0f, 1.0f, 30.0f};
  /* Conservative starting values, still subject to every configured limit. */
  (void)m;
  *c = defaults;
}

bool foc_ident_active(const motor_handle_t *m) { return m && owner == m; }
/* Called in ISR or under PRIMASK. Clear owner before the disarm callback. */
static void finish(motor_handle_t *m, ident_error_t why) {
  if (owner != m) return;
  if (ident_engine_active(&engine)) ident_engine_abort(&engine, why);
  owner = NULL;
  motor_disarm(m);
  m->state.v_d = m->state.v_q = 0;
  if (!m->fault.latched && m->fsm != STATE_FAULT) foc_fsm_next_state(m, STATE_IDLE);
}
void foc_ident_on_disarm(motor_handle_t *m) {
  uint32_t irq;
  if (!foc_ident_active(m)) return;
  irq = __get_PRIMASK(); __disable_irq();
  ident_engine_abort(&engine, m->fault.latched ? IDENT_EXTERNAL : IDENT_OK);
  owner = NULL;
  if (!m->fault.latched && m->fsm != STATE_FAULT) foc_fsm_next_state(m, STATE_IDLE);
  __set_PRIMASK(irq);
}
void foc_ident_stop(motor_handle_t *m) {
  uint32_t irq;
  if (!foc_ident_active(m)) return;
  irq = __get_PRIMASK(); __disable_irq(); finish(m, IDENT_OK); __set_PRIMASK(irq);
}
static error_t admission(motor_handle_t *m, const ident_config_t *c) {
  motor_runtime_t *rt = motor_runtime(m);
  float bus = m->state.vbus_filtered;
  if (!rt || m != motor_get_handler()) return ERR_PARAM;
  if (m->fault.latched || m->fault.status || m->fsm == STATE_FAULT) return ERR_FAULT_LATCHED;
  if (owner || m->fsm != STATE_IDLE || motor_is_armed(m)) return ERR_STATE;
  if (!m->current_offset_cal_done) return ERR_OFFSET_CAL;
  if (!rt->electrical_align_done) return ERR_ALIGIN_FAILED;
  if (m->config.param.direction != DIR_CW && m->config.param.direction != DIR_CCW) return ERR_PARAM;
  if (!finite_f(bus) || !(bus > 1) || !finite_f(m->state.shaft.velocity_rad_s) ||
      fabsf(m->state.shaft.velocity_rad_s) > 0.5f) return ERR_STATE;
  if (!(c->current_trip_a <= m->config.limits.id_limit) ||
      !(c->current_trip_a <= m->config.limits.iq_limit) ||
      (m->config.limits.sw_ocp > 0 && c->current_trip_a > m->config.limits.sw_ocp) ||
      !(c->voltage_v <= m->config.limits.vd_limit) || !(c->voltage_v <= m->config.limits.vq_limit) ||
      !(c->voltage_v < 0.9f*bus*AT32_FOC_CURRENT_MAX_DUTY*ONE_DIV_SQRT_3) ||
      !(c->speed_trip_rad_s < MAX_VELOCITY_RAD_S)) return ERR_PARAM;
  return ERR_NONE;
}
error_t foc_ident_start(motor_handle_t *m, ident_mode_t mode, const ident_config_t *cfg) {
  ident_config_t c;
  ident_engine_t prepared;
  ident_result_t seed;
  error_t err;
  uint32_t irq;
  if (!m) return ERR_NULL;
  if (mc_kth71_cal_active()) return ERR_CALIBRATING;
  if (cfg) c = *cfg; else foc_ident_defaults(m, &c);
  err = admission(m, &c); if (err != ERR_NONE) return err;
  seed.rs = m->config.param.phase_resistance; seed.ld = m->config.param.inductance_d;
  seed.lq = m->config.param.inductance_q; seed.flux = 0; seed.valid = 0;
  if (!ident_engine_start(&prepared, &c, mode, FOC_CURRENT_PERIOD_S,
                          m->config.param.pole_pairs, &seed, m->state.shaft.position_rad)) return ERR_PARAM;
  irq = __get_PRIMASK(); __disable_irq();
  err = admission(m, &c);
  if (err == ERR_NONE) {
    engine = prepared; original = m->config.param;
    have_previous = false;
    foc_fsm_next_state(m, mode == IDENT_FLUX ? STATE_FLUX_CALIBRATION : STATE_RSLS_CALIBRATION);
  }
  __set_PRIMASK(irq);
  if (err != ERR_NONE) return err;
  /* Driver wake-up can sleep for milliseconds: keep it OUT of the ADC ISR
   * and outside the IRQ mask. The calibration FSM reserves PWM ownership. */
  m->out.duty_a = m->out.duty_b = m->out.duty_c = 0;
  motor_arm(m);
  irq = __get_PRIMASK(); __disable_irq();
  if (!motor_is_armed(m) || m->fault.latched ||
      (m->fsm != STATE_RSLS_CALIBRATION && m->fsm != STATE_FLUX_CALIBRATION)) {
    ident_engine_abort(&engine, IDENT_EXTERNAL);
    motor_disarm(m);
    if (!m->fault.latched && m->fsm != STATE_FAULT) foc_fsm_next_state(m, STATE_IDLE);
    err = ERR_STATE;
  } else {
    __DMB(); owner = m;
  }
  __set_PRIMASK(irq); return err;
}
void foc_ident_status(foc_ident_status_t *s) {
  uint32_t irq;
  if (!s) return;
  irq = __get_PRIMASK(); __disable_irq();
  s->stage = engine.stage; s->error = engine.error; s->result = engine.result;
  s->elapsed_ms = (uint32_t)((float)engine.total*engine.dt*1000.0f);
  __set_PRIMASK(irq);
}
error_t foc_ident_apply(motor_handle_t *m) {
  uint32_t irq;
  error_t ret = ERR_STATE;
  if (!m || m != motor_get_handler()) return ERR_NULL;
  irq = __get_PRIMASK(); __disable_irq();
  if (!owner && engine.stage == IDENT_DONE && engine.result.valid &&
      m->fsm == STATE_IDLE && !motor_is_armed(m) && !m->fault.latched &&
      memcmp(&original, &m->config.param, sizeof(original)) == 0) {
    if (engine.result.valid & 1u) {
      m->config.param.phase_resistance = engine.result.rs;
      m->config.param.inductance_d = engine.result.ld;
      m->config.param.inductance_q = engine.result.lq;
    }
    if (engine.result.valid & 2u) m->config.param.flux_linkage = engine.result.flux;
    original = m->config.param; ret = ERR_NONE;
  }
  __set_PRIMASK(irq); return ret;
}
void foc_ident_run_isr(motor_handle_t *m) {
  ident_sample_t s;
  ident_output_t out;
  float bus, sn, cs, alpha, beta;
  uint32_t now;
  if (!foc_ident_active(m)) return;
  if (m->fault.latched || (m->fsm != STATE_RSLS_CALIBRATION && m->fsm != STATE_FLUX_CALIBRATION)) {
    finish(m, IDENT_EXTERNAL); return;
  }
  now = dwt_get_ticks();
  /* Subtract raw cycles BEFORE converting: CYCCNT wraps every ~24 seconds. */
  if (have_previous && (dwt_ticks_to_us(now-previous_ticks) < (uint32_t)(0.5f*FOC_CURRENT_PERIOD_S*1e6f) ||
      dwt_ticks_to_us(now-previous_ticks) > (uint32_t)(1.5f*FOC_CURRENT_PERIOD_S*1e6f))) {
    finish(m, IDENT_TIMING); return;
  }
  previous_ticks = now; have_previous = true;
  bus = m->state.vbus_filtered;
  if (!finite_f(bus) || !(bus > 1) ||
      (m->config.limits.vbus_uv_threshold > 0 && bus < m->config.limits.vbus_uv_threshold) ||
      (m->config.limits.vbus_ov_threshold > 0 && bus > m->config.limits.vbus_ov_threshold) ||
      !(engine.cfg.voltage_v < 0.9f*bus*AT32_FOC_CURRENT_MAX_DUTY*ONE_DIV_SQRT_3)) {
    finish(m, IDENT_VOLTAGE); return;
  }
  s.id = m->state.i_d; s.iq = m->state.i_q;
  s.speed = m->state.shaft.velocity_rad_s; s.position = m->state.shaft.position_rad;
  s.phase_peak = 0;
  for (unsigned i = 0; i < 3; i++) {
    float a = m->state.phase_current.ampere[i];
    if (!finite_f(a)) { finish(m, IDENT_NONFINITE); return; }
    s.phase_peak = FMAX(s.phase_peak, fabsf(a));
  }
  if (!finite_f(m->state.theta_elec_rad)) { finish(m, IDENT_NONFINITE); return; }
  /* Duty differences remove SVM common mode. Actual bus and post-limit PWM,
   * not the pre-saturation PID request, determine the voltage estimate. */
  alpha = (2*m->out.duty_a-m->out.duty_b-m->out.duty_c)*bus/3.0f;
  beta = (m->out.duty_b-m->out.duty_c)*bus*ONE_DIV_SQRT_3;
  foc_sincos_shared(m->state.theta_elec_rad, &sn, &cs);
  s.vd = alpha*cs+beta*sn; s.vq = -alpha*sn+beta*cs;
  out = ident_engine_step(&engine, &s);
  if (!ident_engine_active(&engine)) { finish(m, engine.error); return; }
  if (!motor_is_armed(m)) { finish(m, IDENT_EXTERNAL); return; }
  if (engine.stage >= IDENT_SPIN && m->fsm != STATE_FLUX_CALIBRATION)
    foc_fsm_next_state(m, STATE_FLUX_CALIBRATION);
  m->state.v_d = out.vd; m->state.v_q = out.vq;
  foc_voltage(m, out.vq, out.vd, m->state.theta_elec_rad);
  if (m->fault.latched || !motor_is_armed(m)) finish(m, IDENT_EXTERNAL);
}
