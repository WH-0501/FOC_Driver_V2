#if defined(__CC_ARM)
#pragma Ospace
#endif
#include "ident_engine.h"
#include "foc_math.h"
#include <float.h>
#include <math.h>
#include <string.h>
#define PI2 6.2831853071795864769f
static bool finite_f(float x) { return x >= -FLT_MAX && x <= FLT_MAX; }
static float clip(float x, float lim) { return x < -lim ? -lim : (x > lim ? lim : x); }
static bool range(float x, float lo, float hi) { return x >= lo && x <= hi; }
bool ident_engine_active(const ident_engine_t *e) {
  return e && e->stage >= IDENT_RS_LOW && e->stage <= IDENT_FLUX_SAMPLE;
}
void ident_engine_abort(ident_engine_t *e, ident_error_t error) {
  if (!ident_engine_active(e)) return;
  e->error = error; e->stage = error == IDENT_OK ? IDENT_CANCELLED : IDENT_FAILED;
  e->result.valid = 0;
}
static void stage(ident_engine_t *e, ident_stage_t next) {
  e->stage = next; e->tick = e->n = 0;
  e->sum_i = e->sum_v = e->sum_i2 = e->sum_v2 = 0;
  e->sum_s = e->sum_c = e->sum_vs = e->sum_vc = 0;
  e->vd_int = e->vq_int = 0;
}
bool ident_engine_start(ident_engine_t *e, const ident_config_t *c, ident_mode_t mode,
                        float dt, unsigned pp, const ident_result_t *seed, float pos) {
  if (!e || !c || !range(dt, 0.00001f, 0.001f) || pp == 0 || pp > 255 ||
      !finite_f(pos) || mode < IDENT_RSLS || mode > IDENT_ALL ||
      !range(c->current_a, 0.02f, 50) || !range(c->voltage_v, 0.05f, 100) ||
      !range(c->injection_v, 0.01f, c->voltage_v) ||
      !range(c->injection_hz, 20, 0.05f / dt) ||
      !range(c->current_trip_a, 1.5f*c->current_a, 100) ||
      !range(c->speed_rad_s, 2, 200) ||
      !range(c->speed_trip_rad_s, 1.25f*c->speed_rad_s, 300)) return false;
  if (mode == IDENT_FLUX && (!seed || !range(seed->rs, 0.0001f, 100) ||
      !range(seed->ld, 0.0000001f, 1) || !range(seed->lq, 0.0000001f, 1))) return false;
  memset(e, 0, sizeof(*e));
  e->cfg = *c; e->mode = mode; e->dt = dt; e->pole_pairs = (float)pp; e->origin = pos;
  e->period = (uint32_t)(1.0f / (dt * c->injection_hz) + 0.5f);
  if (mode == IDENT_FLUX) e->result = *seed;
  e->result.valid = 0;
  stage(e, mode == IDENT_FLUX ? IDENT_SPIN : IDENT_RS_LOW);
  return true;
}
static ident_output_t fail(ident_engine_t *e, ident_error_t why) {
  ident_output_t zero = {0, 0}; ident_engine_abort(e, why); return zero;
}
static void finish_rs(ident_engine_t *e) {
  float i = e->sum_i / (float)e->n, v = e->sum_v / (float)e->n;
  float target = e->cfg.current_a * (e->stage == IDENT_RS_LOW ? 0.5f : 1.0f);
  float variance = e->sum_i2 / (float)e->n - i*i;
  if (fabsf(i-target) > 0.05f*target || variance > 0.0025f*target*target) {
    ident_engine_abort(e, IDENT_SIGNAL); return;
  }
  if (e->stage == IDENT_RS_LOW) {
    e->low_i = i; e->low_v = v; stage(e, IDENT_RS_HIGH);
  } else {
    float r = (v-e->low_v)/(i-e->low_i);
    if (!range(r, 0.0001f, 100)) { ident_engine_abort(e, IDENT_SIGNAL); return; }
    e->result.rs = r;
    e->amplitude = FMIN(e->cfg.injection_v, 0.5f*r*e->cfg.current_a);
    stage(e, IDENT_LD);
  }
}
static void finish_l(ident_engine_t *e) {
  float n = (float)e->n;
  float i2 = 4.0f*(e->sum_s*e->sum_s + e->sum_c*e->sum_c)/(n*n);
  float v2 = 4.0f*(e->sum_vs*e->sum_vs + e->sum_vc*e->sum_vc)/(n*n);
  float variance = e->sum_i2/n - (e->sum_i/n)*(e->sum_i/n);
  float z2, l, omega = PI2/(e->dt*(float)e->period);
  /* Magnitude impedance avoids dependence on PWM/ADC phase delay. Require
   * a measurable reactive component and predominantly sinusoidal response. */
  if (!(i2 > 0.000025f) || variance > 0.65f*i2 || !(v2 > 0.00000001f)) {
    ident_engine_abort(e, IDENT_SIGNAL); return;
  }
  z2 = v2/i2;
  if (!(z2 > 1.04f*e->result.rs*e->result.rs)) {
    ident_engine_abort(e, IDENT_SIGNAL); return;
  }
  l = sqrtf(z2-e->result.rs*e->result.rs)/omega;
  if (!range(l, 0.0000001f, 1)) { ident_engine_abort(e, IDENT_SIGNAL); return; }
  if (e->stage == IDENT_LD) { e->result.ld = l; stage(e, IDENT_LQ); }
  else {
    e->result.lq = l;
    if (e->mode == IDENT_ALL) stage(e, IDENT_SPIN);
    else { e->result.valid = 1; stage(e, IDENT_DONE); }
  }
}
ident_output_t ident_engine_step(ident_engine_t *e, const ident_sample_t *s) {
  ident_output_t out = {0, 0};
  float time, magnitude;
  if (!ident_engine_active(e)) return out;
  if (!s || !finite_f(s->id) || !finite_f(s->iq) || !finite_f(s->speed) ||
      !finite_f(s->position) || !finite_f(s->phase_peak) ||
      !finite_f(s->vd) || !finite_f(s->vq)) return fail(e, IDENT_NONFINITE);
  if (s->phase_peak >= e->cfg.current_trip_a ||
      s->id*s->id + s->iq*s->iq >= e->cfg.current_trip_a*e->cfg.current_trip_a)
    return fail(e, IDENT_CURRENT);
  if (fabsf(s->speed) >= e->cfg.speed_trip_rad_s) return fail(e, IDENT_MOTION);
  if (e->stage <= IDENT_LQ &&
      (fabsf(s->position-e->origin)*e->pole_pairs > 0.12f || fabsf(s->speed) > 1.0f))
    return fail(e, IDENT_MOTION);
  if (++e->total * e->dt > 30.0f) return fail(e, IDENT_TIMEOUT);
  time = (float)e->tick * e->dt;
  if (e->stage == IDENT_RS_LOW || e->stage == IDENT_RS_HIGH) {
    float target = e->cfg.current_a * (e->stage == IDENT_RS_LOW ? 0.5f : 1.0f);
    /* Voltage integrator works without pre-existing motor/PID parameters. */
    e->vd_int = clip(e->vd_int + 5.0f*e->cfg.voltage_v/e->cfg.current_a *
                    (target-s->id)*e->dt, e->cfg.voltage_v);
    out.vd = e->vd_int;
    if (time >= 0.8f) {
      e->sum_i += s->id; e->sum_i2 += s->id*s->id; e->sum_v += s->vd; e->n++;
    }
    if (time >= 1.0f) { finish_rs(e); out.vd = 0; return out; }
  } else if (e->stage == IDENT_LD || e->stage == IDENT_LQ) {
    float phase = PI2 * (float)(e->tick % e->period)/(float)e->period;
    float sn, cs;
    foc_sincos_shared(phase, &sn, &cs);
    float i = e->stage == IDENT_LD ? s->id : s->iq;
    float v = e->stage == IDENT_LD ? s->vd : s->vq;
    /* Integer cycles: 32 settle + 64 measure. Both voltage and current are
     * demodulated at the SAME sample instant; constant offsets cancel. */
    if (e->tick >= 32u*e->period) {
      e->sum_s += i*sn; e->sum_c += i*cs;
      e->sum_vs += v*sn; e->sum_vc += v*cs;
      e->sum_i += i; e->sum_i2 += i*i; e->n++;
    }
    if (e->stage == IDENT_LD) out.vd = e->amplitude*sn;
    else out.vq = e->amplitude*sn;
    if (e->tick + 1u >= 96u*e->period) { finish_l(e); return (ident_output_t){0, 0}; }
  } else {
    float ramp = FMIN(1.0f, time/2.0f);
    float target = e->cfg.speed_rad_s*(e->stage == IDENT_SPIN ? ramp : 1.0f);
    float iq_ref = clip(5.0f*e->cfg.current_a*(target-s->speed)/e->cfg.speed_rad_s,
                        e->cfg.current_a);
    float bandwidth = FMIN(PI2*200.0f, 0.1f/e->dt);
    float vd, vq, scale = 1.0f;
    e->vd_int += bandwidth*e->result.rs*(-s->id)*e->dt;
    e->vq_int += bandwidth*e->result.rs*(iq_ref-s->iq)*e->dt;
    e->vd_int = clip(e->vd_int, e->cfg.voltage_v);
    e->vq_int = clip(e->vq_int, e->cfg.voltage_v);
    vd = -bandwidth*e->result.ld*s->id + e->vd_int;
    vq = bandwidth*e->result.lq*(iq_ref-s->iq) + e->vq_int;
    magnitude = sqrtf(vd*vd + vq*vq);
    if (magnitude > e->cfg.voltage_v) scale = e->cfg.voltage_v/magnitude;
    out.vd = vd*scale; out.vq = vq*scale;
    /* Back-calculation prevents integrator wind-up at the vector limit. */
    e->vd_int += out.vd-vd; e->vq_int += out.vq-vq;
    if (e->stage == IDENT_SPIN) {
      if (time > 8.0f) return fail(e, IDENT_TIMEOUT);
      if (time >= 3.0f && fabsf(s->speed-e->cfg.speed_rad_s) < 0.1f*e->cfg.speed_rad_s
          && scale == 1.0f) e->n++; else e->n = 0;
      if ((float)e->n*e->dt >= 0.5f) {
        /* Preserve PI state across the sampling boundary. */
        e->stage = IDENT_FLUX_SAMPLE; e->tick = 0; e->n = 0;
        e->first_iq = s->iq; e->flux_num = e->flux_den = 0;
        e->min_speed = e->max_speed = s->speed;
        return out;
      }
    } else {
      float omega = s->speed*e->pole_pairs;
      if (scale < 1.0f || fabsf(s->speed-e->cfg.speed_rad_s) > 0.15f*e->cfg.speed_rad_s)
        return fail(e, IDENT_SIGNAL);
      e->min_speed = FMIN(e->min_speed, s->speed); e->max_speed = FMAX(e->max_speed, s->speed);
      e->flux_num += (s->vq-e->result.rs*s->iq-omega*e->result.ld*s->id)*e->dt;
      e->flux_den += omega*e->dt;
      if (time >= 1.0f) {
        float flux = (e->flux_num-e->result.lq*(s->iq-e->first_iq))/e->flux_den;
        if (!range(flux, 0.000001f, 10) || e->max_speed-e->min_speed > 0.1f*e->cfg.speed_rad_s)
          return fail(e, IDENT_SIGNAL);
        e->result.flux = flux; e->result.valid = e->mode == IDENT_ALL ? 3 : 2;
        stage(e, IDENT_DONE); return (ident_output_t){0, 0};
      }
    }
  }
  e->tick++;
  if (!ident_engine_active(e)) return (ident_output_t){0, 0};
  return out;
}
