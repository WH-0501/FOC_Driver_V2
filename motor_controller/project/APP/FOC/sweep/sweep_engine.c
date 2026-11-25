#include "sweep_engine.h"
#include <float.h>
#include "sweep_engine_internal.h"
#include "sweep_phase_seeds.inc"

#if defined(__CC_ARM)
__attribute__((noinline))
#endif
static int finite_float(float f) { return f >= -FLT_MAX && f <= FLT_MAX; }
/* Keep the existing shared call in the ISR when foreground callers change. */
#if defined(__CC_ARM)
__attribute__((noinline))
#endif
void sweep_engine_reset_phase(sweep_engine_t *e, unsigned i)
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
  sweep_engine_reset_phase(e, e->index);
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
  /* Settle produces the identical chirp but no spectrum is consumed. Keep
   * oscillators at the phase required by the first measurement sample. */
  if (!measuring) {
    if (phase_reset) for (i=0; i<e->count; ++i) { e->tone[i].s=0; e->tone[i].c=1; }
  } else if (!block_end) {
    /* Hot path: hoist all stage tests out of the per-frequency loop.
     * Preserve each float operation and its accumulation order. */
    sweep_tone_t *t=e->tone, *end=t+e->count;
    /* Cache the original phase and walk by pointer: avoid per-bin index
     * multiplication, repeated count loads and reloading phase after stores.
     * Same binary32 operations/order within each accumulator and oscillator. */
    for (; t!=end; ++t) {
      float s=t->s, c=t->c, ds=t->ds, dc=t->dc;
      float yr=t->yr + y*c, yi=t->yi - y*s;
      float sn=s*dc + c*ds, cn=c*dc - s*ds;
      t->yr=yr; t->yi=yi; t->s=sn; t->c=cn;
    }
  } else {
    for (i=0; i<e->count; ++i) {
      sweep_tone_t *t=&e->tone[i];
      float yr,yi;
      t->yr += y*t->c; t->yi -= y*t->s;
      yr=t->yr; yi=t->yi;
      t->yy += yr*yr + yi*yi;
      t->sum_r += yr; t->sum_i += yi;
      t->yr=t->yi=t->s=0; t->c=1;
    }
  }
  ++e->samples;
  if (e->remaining) --e->remaining;
  else if (--e->block_remaining == 0u) {
    e->block_remaining=e->length;
    if (++e->blocks == 3u) { e->progress=e->count; e->state=SWEEP_DONE; }
  }
  return x;
}

/**
 * @brief 该函数是扫频引擎的步函数，用于执行扫频操作。
 * 
 * @param e 
 * @param y 
 * @return float 
 */
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
    if (phase_reset) sweep_engine_reset_phase(e, i);
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
