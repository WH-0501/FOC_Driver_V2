/* Foreground setup and H1 reduction. Keep speed optimization in sweep_engine.c.
 * Arithmetic, accumulation order and double-precision statistics are unchanged. */
#if defined(__CC_ARM)
#pragma Ospace
#endif
#include "sweep_engine_internal.h"
#include <float.h>
#include <math.h>
#include <string.h>
#define SWEEP_PI 3.14159265358979323846f
static int finite_float(float f) { return f >= -FLT_MAX && f <= FLT_MAX; }
static void rotate(sweep_tone_t *t)
{
  float s = t->s * t->dc + t->c * t->ds;
  t->c = t->c * t->dc - t->s * t->ds;
  t->s = s;
}
static void select_point(sweep_engine_t *e)
{
  const sweep_tone_t *t = &e->tone[e->index];
  if (e->method == SWEEP_STEPPED_SINE) e->length = (uint16_t)(t->bin_or_period * 4u);
  e->remaining = t->settle;
  e->block_remaining = e->length;
  e->blocks = 0u;
  sweep_engine_reset_phase(e, e->index);
}
/* Foreground only: the grid needs positive integer roots (degree <= 15),
 * not general powf. Bisection retains double precision and removes its library.
 * Endpoints below are explicit, independent of root rounding. */
static double geometric_step(float low, float high, unsigned degree)
{
  double target = (double)high / low, lo = 1.0, hi = target;
  unsigned iter, k;
  for (iter = 0u; iter < 48u; ++iter) {
    double mid = (lo + hi) * 0.5, power = 1.0;
    for (k = 0u; k < degree; ++k) power *= mid;
    if (power < target) lo = mid; else hi = mid;
  }
  return (lo + hi) * 0.5;
}
int sweep_engine_prepare(sweep_engine_t *e, const sweep_config_t *cfg)
{
  uint16_t bins[SWEEP_MAX_POINTS];
  float last = 0.0f, maxsum = 0.0f;
  unsigned i, j, period_samples, anchors;
  double step, next;
  if (!e || !cfg) return 2;
  if (e->state == SWEEP_RUNNING) return 1;
  if ((cfg->method != SWEEP_MULTISINE && cfg->method != SWEEP_STEPPED_SINE && cfg->method != SWEEP_CHIRP) ||
      cfg->count < 3u || cfg->count > SWEEP_MAX_POINTS ||
      !finite_float(cfg->low_hz) || !finite_float(cfg->high_hz) ||
      !finite_float(cfg->peak) || !finite_float(cfg->sample_hz) ||
      cfg->sample_hz < 10000.0f || cfg->sample_hz > 40000.0f ||
      cfg->low_hz < 20.0f || cfg->high_hz <= cfg->low_hz ||
      cfg->high_hz > 2000.0f || cfg->high_hz > cfg->sample_hz * 0.1f ||
      cfg->peak < 0.005f || cfg->peak > 0.5f ||
      (cfg->low_hz > 40.0f && cfg->count < 4u)) return 2;

  anchors = cfg->low_hz > 40.0f ? 2u : 0u;
  step = geometric_step(cfg->low_hz, cfg->high_hz, cfg->count - anchors - 1u);
  next = cfg->low_hz;
  period_samples = (unsigned)(cfg->sample_hz * (cfg->method == SWEEP_CHIRP ? 1.0f : 0.05f) + 0.5f);
  /* Validate all quantized frequencies before overwriting an earlier result. */
  for (i = 0u; i < cfg->count; ++i) {
    float f;
    if (i < anchors) f = i ? 40.0f : 20.0f;
    else {
      f = i == cfg->count - 1u ? cfg->high_hz : (float)next;
      next *= step;
    }
    if (cfg->method != SWEEP_STEPPED_SINE) {
      float grid = cfg->sample_hz / period_samples;
      unsigned bin = (unsigned)(f / grid + 0.5f);
      if (i && bin <= bins[i-1u]) bin = bins[i-1u] + 1u;
      bins[i] = (uint16_t)bin;
      f = bin * grid;
    } else {
      /* Positive finite quotient: exact integer ceil, without a math call. */
      float samples = cfg->sample_hz / f;
      bins[i] = (uint16_t)samples;
      if ((float)bins[i] < samples) ++bins[i];
      f = cfg->sample_hz / bins[i];
    }
    if (f <= last || f > cfg->high_hz + 0.001f) return 2;
    last = f;
  }
  memset(e, 0, sizeof(*e));
  e->method = (uint8_t)cfg->method;
  e->count = (uint8_t)cfg->count;
  e->sample_hz = cfg->sample_hz;
  e->scale = cfg->peak;
  e->length = (uint16_t)period_samples;
  for (i = 0u; i < e->count; ++i) {
    sweep_tone_t *t = &e->tone[i];
    float f;
    t->bin_or_period = bins[i];
    if (e->method != SWEEP_STEPPED_SINE) t->settle = (uint16_t)period_samples;
    else {
      unsigned period = bins[i], length = period * 4u;
      unsigned settling = (unsigned)(cfg->sample_hz * 0.02f);
      if (settling < length) settling = length;
      t->settle = (uint16_t)(((settling + period - 1u) / period) * period);
    }
    f = sweep_engine_frequency(e, i);
    t->ds = sinf(2.0f * SWEEP_PI * f / cfg->sample_hz);
    t->dc = cosf(2.0f * SWEEP_PI * f / cfg->sample_hz);
    sweep_engine_reset_phase(e, i);
  }
  if (e->method == SWEEP_MULTISINE) {
    /* Precompute the discrete waveform peak in foreground, interrupts enabled.
     * ISR uses the same recurrence, reset at each coherent block. */
    for (j = 0u; j < period_samples; ++j) {
      float sum = 0.0f;
      for (i = 0u; i < e->count; ++i) { sum += e->tone[i].s; rotate(&e->tone[i]); }
      if (fabsf(sum) > maxsum) maxsum = fabsf(sum);
    }
    e->scale = cfg->peak / (maxsum * 1.001f);
    for (i = 0u; i < e->count; ++i) sweep_engine_reset_phase(e, i);
  }
  if (e->method == SWEEP_CHIRP) {
    float maximum = sweep_chirp_prepare(&e->chirp, cfg->low_hz > 40.0f ? 20.0f : cfg->low_hz,
                        cfg->high_hz, cfg->sample_hz, e->length);
    e->scale = cfg->peak / (maximum * 1.001f);
  }
  /* All modes collect the exact generated input against the same recurrence.
   * Share foreground DFT code; preserve each mode's loop order and scaling.
   * Stepped sine has one tone per group; multisine/chirp share one group. */
  {
    unsigned group, groups = e->method == SWEEP_STEPPED_SINE ? e->count : 1u;
    for (group = 0u; group < groups; ++group) {
      unsigned first = e->method == SWEEP_STEPPED_SINE ? group : 0u;
      unsigned end = e->method == SWEEP_STEPPED_SINE ? group + 1u : e->count;
      unsigned length = e->method == SWEEP_STEPPED_SINE ?
          4u * e->tone[group].bin_or_period : period_samples;
      for (j = 0u; j < length; ++j) {
        float x;
        if (e->method == SWEEP_CHIRP)
          x = sweep_chirp_step(&e->chirp, e->length, j) * e->scale;
        else if (e->method == SWEEP_STEPPED_SINE)
          x = e->tone[group].s * e->scale;
        else {
          x = 0.0f;
          for (i = first; i < end; ++i) x += e->tone[i].s * e->scale;
        }
        for (i = first; i < end; ++i) {
          sweep_tone_t *t = &e->tone[i];
          t->xr += x * t->c; t->xi -= x * t->s; rotate(t);
        }
      }
      for (i = first; i < end; ++i) {
        e->tone[i].xr *= 2.0f / length;
        e->tone[i].xi *= 2.0f / length;
        sweep_engine_reset_phase(e, i);
      }
    }
  }
  if (e->method == SWEEP_CHIRP) sweep_chirp_reset(&e->chirp);
  select_point(e);
  return 0;
}
int sweep_engine_result(const sweep_engine_t *e, unsigned index, sweep_result_t *out)
{
  const sweep_tone_t *t;
  float xx, blocks;
  double norm, den;
  if (!e || !out || e->state != SWEEP_DONE || index >= e->count) return 1;
  t = &e->tone[index];
  memset(out, 0, sizeof(*out));
  out->hz = sweep_engine_frequency(e, index);
  out->blocks = e->method != SWEEP_STEPPED_SINE ? 3u : 4u;
  blocks = (float)out->blocks;
  xx = t->xr * t->xr + t->xi * t->xi;
  if (!(xx > 1e-15f) || !finite_float(xx) || !finite_float(t->yy) ||
      !finite_float(t->sum_r) || !finite_float(t->sum_i)) return 2;
  norm = 2.0 / (e->method != SWEEP_STEPPED_SINE ? e->length : 4u * t->bin_or_period);
  den = (double)blocks * xx;
  out->input = sqrtf(xx); out->output = (float)(sqrtf(t->yy / blocks) * norm);
  out->real = (float)(((double)t->sum_r * t->xr + (double)t->sum_i * t->xi) * norm / den);
  out->imag = (float)(((double)t->sum_i * t->xr - (double)t->sum_r * t->xi) * norm / den);
  /* Identical X: gamma squared = |sum(Y)| squared / (N*sum(|Y| squared)). */
  if (t->yy > 1e-15f) out->coherence =
      (float)(((double)t->sum_r * t->sum_r + (double)t->sum_i * t->sum_i) / ((double)blocks * t->yy));
  if (out->coherence > 1.0f) out->coherence = 1.0f;
  return 0;
}

float sweep_chirp_prepare(sweep_chirp_t *s, float low, float high, float fs, uint32_t length)
{
  uint32_t last, difference, i;
  double sum = 0.0, weight = 0.0;
  float maximum = 0.0f;
  memset(s, 0, sizeof(*s));
  s->first_step = (uint32_t)((double)low * 4294967296.0 / fs + 0.5);
  last = (uint32_t)((double)high * 4294967296.0 / fs + 0.5);
  difference = last - s->first_step;
  s->delta = difference / (length - 1u);
  s->remainder = difference % (length - 1u);
  s->ramp_gain = 200.0f / fs; /* smooth 5 ms edges, within each 1 s period */
  sweep_chirp_reset(s);
  for (i = 0; i < length; ++i) {
    uint32_t edge = i < length-1u-i ? i : length-1u-i;
    float u = (float)edge * s->ramp_gain;
    weight += u < 1.0f ? u*u*(3.0f-2.0f*u) : 1.0f;
    sum += sweep_chirp_step(s, length, i);
  }
  s->dc = (float)(sum / weight); /* zero mean, without nonzero endpoint steps */
  sweep_chirp_reset(s);
  for (i = 0; i < length; ++i) {
    float x = sweep_chirp_step(s, length, i);
    if (x < 0.0f) x = -x;
    if (x > maximum) maximum = x;
  }
  sweep_chirp_reset(s);
  return maximum;
}
