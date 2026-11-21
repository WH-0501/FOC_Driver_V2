#include "sweep_engine.h"
#include <float.h>
#include <math.h>
#include <string.h>
#include "sweep_phase_seeds.inc"

#define SWEEP_PI 3.14159265358979323846f
static int finite_float(float f) { return f >= -FLT_MAX && f <= FLT_MAX; }
static void reset_phase(sweep_engine_t *e, unsigned i)
{
  sweep_tone_t *t = &e->tone[i];
  if (e->method != SWEEP_MULTISINE || i < 2u) { t->s = 0.0f; t->c = 1.0f; }
  else {
    unsigned seed = (e->count - 3u) * (e->count - 2u) / 2u + i - 2u;
    t->s = sweep_phase_seeds[seed][0]; t->c = sweep_phase_seeds[seed][1];
  }
}
static void rotate(sweep_tone_t *t)
{
  float s = t->s * t->dc + t->c * t->ds;
  t->c = t->c * t->dc - t->s * t->ds;
  t->s = s;
}
float sweep_engine_frequency(const sweep_engine_t *e, unsigned index)
{
  if (!e || index >= e->count) return 0.0f;
  if (e->method == SWEEP_STEPPED_SINE) return e->sample_hz / e->tone[index].bin_or_period;
  if (e->method == SWEEP_CHIRP) return e->tone[index].bin_or_period * (e->sample_hz / e->length);
  return e->tone[index].bin_or_period * (e->sample_hz / (unsigned)(e->sample_hz * 0.05f + 0.5f));
}
static void select_point(sweep_engine_t *e)
{
  const sweep_tone_t *t = &e->tone[e->index];
  if (e->method == SWEEP_STEPPED_SINE) e->length = (uint16_t)(t->bin_or_period * 4u);
  e->remaining = t->settle;
  e->block_remaining = e->length;
  e->blocks = 0u;
  reset_phase(e, e->index);
}
int sweep_engine_prepare(sweep_engine_t *e, const sweep_config_t *cfg)
{
  uint16_t bins[SWEEP_MAX_POINTS];
  float last = 0.0f, maxsum = 0.0f;
  unsigned i, j, period_samples;
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

  period_samples = (unsigned)(cfg->sample_hz * (cfg->method == SWEEP_CHIRP ? 1.0f : 0.05f) + 0.5f);
  /* Validate all quantized frequencies before overwriting an earlier result. */
  for (i = 0u; i < cfg->count; ++i) {
    float f;
    if (cfg->low_hz > 40.0f && i < 2u) f = i ? 40.0f : 20.0f;
    else if (cfg->low_hz > 40.0f)
      f = cfg->low_hz * powf(cfg->high_hz / cfg->low_hz, (float)(i - 2u) / (cfg->count - 3u));
    else f = cfg->low_hz * powf(cfg->high_hz / cfg->low_hz, (float)i / (cfg->count - 1u));
    if (cfg->method != SWEEP_STEPPED_SINE) {
      float grid = cfg->sample_hz / period_samples;
      unsigned bin = (unsigned)floorf(f / grid + 0.5f);
      if (i && bin <= bins[i-1u]) bin = bins[i-1u] + 1u;
      bins[i] = (uint16_t)bin;
      f = bin * grid;
    } else {
      bins[i] = (uint16_t)ceilf(cfg->sample_hz / f);
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
    reset_phase(e, i);
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
    for (i = 0u; i < e->count; ++i) reset_phase(e, i);
  }
  if (e->method == SWEEP_CHIRP) {
    float maximum = sweep_chirp_prepare(&e->chirp, cfg->low_hz > 40.0f ? 20.0f : cfg->low_hz,
                        cfg->high_hz, cfg->sample_hz, e->length);
    e->scale = cfg->peak / (maximum * 1.001f);
    for (j = 0u; j < e->length; ++j) {
      float x = sweep_chirp_step(&e->chirp, e->length, j) * e->scale;
      for (i = 0u; i < e->count; ++i) {
        sweep_tone_t *t = &e->tone[i];
        t->xr += x * t->c; t->xi -= x * t->s; rotate(t);
      }
    }
    for (i = 0u; i < e->count; ++i) {
      e->tone[i].xr *= 2.0f / e->length; e->tone[i].xi *= 2.0f / e->length;
      reset_phase(e, i);
    }
    sweep_chirp_reset(&e->chirp);
    select_point(e);
    return 0;
  }
  /* X is identical in all coherent blocks. Precompute it from the exact
   * finite-precision waveform/basis, including cross-tone leakage; not X=-j*A.
   * All this work is foreground, with interrupts enabled. */
  if (e->method == SWEEP_MULTISINE) {
    for (j = 0u; j < period_samples; ++j) {
      float x = 0.0f;
      for (i = 0u; i < e->count; ++i) x += e->tone[i].s * e->scale;
      for (i = 0u; i < e->count; ++i) {
        sweep_tone_t *t = &e->tone[i];
        t->xr += x * t->c; t->xi -= x * t->s; rotate(t);
      }
    }
    for (i = 0u; i < e->count; ++i) {
      e->tone[i].xr *= 2.0f / period_samples; e->tone[i].xi *= 2.0f / period_samples;
      reset_phase(e, i);
    }
  } else {
    for (i = 0u; i < e->count; ++i) {
      sweep_tone_t *t = &e->tone[i];
      unsigned length = 4u * t->bin_or_period;
      for (j = 0u; j < length; ++j) {
        float x = t->s * e->scale;
        t->xr += x * t->c; t->xi -= x * t->s; rotate(t);
      }
      t->xr *= 2.0f / length; t->xi *= 2.0f / length;
      reset_phase(e, i);
    }
  }
  select_point(e);
  return 0;
}
void sweep_engine_start(sweep_engine_t *e)
{
  if (e && e->state == SWEEP_IDLE && e->count >= 3u) e->state = SWEEP_RUNNING;
}
/* Separate mode-specific ISR path: original multisine/stepped loops do not
 * acquire an extra per-tone chirp branch. All modes share the same statistics
 * layout and foreground H1 reduction. */
static float step_chirp(sweep_engine_t *e, float y)
{
  unsigned i;
  int measuring = e->remaining == 0u;
  int block_end = measuring && e->block_remaining == 1u;
  int phase_reset = block_end || e->remaining == 1u;
  unsigned n = e->length - (measuring ? e->block_remaining : e->remaining);
  float x = e->scale * sweep_chirp_step(&e->chirp, e->length, n);
  if (phase_reset) sweep_chirp_reset(&e->chirp);
  for (i = 0u; i < e->count; ++i) {
    sweep_tone_t *t = &e->tone[i];
    if (measuring) { t->yr += y*t->c; t->yi -= y*t->s; }
    if (block_end) {
      float yr=t->yr, yi=t->yi;
      t->yy += yr*yr + yi*yi;
      t->sum_r += yr; t->sum_i += yi;
      t->yr = t->yi = 0.0f;
    }
    if (phase_reset) { t->s=0.0f; t->c=1.0f; }
    else rotate(t);
  }
  ++e->samples;
  if (e->remaining) --e->remaining;
  else if (--e->block_remaining == 0u) {
    e->block_remaining=e->length;
    if (++e->blocks == 3u) { e->progress=e->count; e->state=SWEEP_DONE; }
  }
  return x;
}
float sweep_engine_step(sweep_engine_t *e, float y)
{
  unsigned i, begin, end;
  int measuring, block_end, phase_reset;
  float x = 0.0f;
  if (!e || e->state != SWEEP_RUNNING) return 0.0f;
  if (!finite_float(y)) { sweep_engine_abort(e, SWEEP_NONFINITE); return 0.0f; }
  if (e->method == SWEEP_CHIRP) return step_chirp(e, y);
  begin = e->method == SWEEP_MULTISINE ? 0u : e->index;
  end = e->method == SWEEP_MULTISINE ? e->count : e->index + 1u;
  measuring = e->remaining == 0u;
  block_end = measuring && e->block_remaining == 1u;
  phase_reset = block_end || e->remaining == 1u;
  /* Consume the old phase for BOTH reference and feedback before resetting it.
   * Fuse block reduction into this traversal, and never rotate a phase that is
   * immediately overwritten. No extra RAM, decimation or change in sum order. */
  for (i = begin; i < end; ++i) {
    sweep_tone_t *t = &e->tone[i];
    x += t->s * e->scale;
    if (measuring) {
      t->yr += y * t->c; t->yi -= y * t->s;
    }
    if (block_end) {
      float yr = t->yr, yi = t->yi;
      t->yy += yr * yr + yi * yi;
      t->sum_r += yr; t->sum_i += yi;
      t->yr = t->yi = 0.0f;
    }
    if (phase_reset) reset_phase(e, i);
    else rotate(t);
  }
  ++e->samples;
  if (e->remaining) {
    --e->remaining;
  } else if (--e->block_remaining == 0u) {
    e->block_remaining = e->length;
    if (++e->blocks >= (e->method == SWEEP_MULTISINE ? 3u : 4u)) {
      if (e->method == SWEEP_MULTISINE || ++e->index == e->count) {
        e->progress = e->count; e->state = SWEEP_DONE;
      } else { e->progress = e->index; select_point(e); }
    }
  }
  return x;
}
void sweep_engine_abort(sweep_engine_t *e, unsigned flags)
{
  if (e && (e->state == SWEEP_RUNNING || e->state == SWEEP_DONE)) {
    e->flags = (uint8_t)(e->flags | flags); e->state = SWEEP_ABORTED;
  }
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
  out->input = sqrtf(xx); out->output = (float)(sqrt((double)t->yy / blocks) * norm);
  out->real = (float)(((double)t->sum_r * t->xr + (double)t->sum_i * t->xi) * norm / den);
  out->imag = (float)(((double)t->sum_i * t->xr - (double)t->sum_r * t->xi) * norm / den);
  /* Identical X: gamma squared = |sum(Y)| squared / (N*sum(|Y| squared)). */
  if (t->yy > 1e-15f) out->coherence =
      (float)(((double)t->sum_r * t->sum_r + (double)t->sum_i * t->sum_i) / ((double)blocks * t->yy));
  if (out->coherence > 1.0f) out->coherence = 1.0f;
  return 0;
}
