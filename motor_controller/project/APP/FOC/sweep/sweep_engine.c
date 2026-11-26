#include "sweep_engine.h"
#include <float.h>
#include <string.h>
static int finite_float(float f) { return f >= -FLT_MAX && f <= FLT_MAX; }
/* Foreground only: round settling time up to complete stimulus cycles. */
static unsigned stepped_cycles(unsigned fs, unsigned period)
{
  unsigned ticks = (fs * SWEEP_STEPPED_SETTLE_MS + 999u) / 1000u;
  unsigned settle = (ticks + period - 1u) / period;
  if (settle < SWEEP_STEPPED_MIN_SETTLE_CYCLES)
    settle = SWEEP_STEPPED_MIN_SETTLE_CYCLES;
  return settle + SWEEP_STEPPED_MEASURE_CYCLES;
}
int sweep_engine_prepare(sweep_engine_t *e, const sweep_config_t *c)
{
  float maximum;
  unsigned i;
  if (!e || !c) return 2;
  if (e->state == SWEEP_RUNNING) return 1;
  if ((c->method != SWEEP_CHIRP && c->method != SWEEP_STEPPED) || c->count < SWEEP_MIN_POINTS || c->count > SWEEP_MAX_POINTS ||
      !finite_float(c->low_hz) || !finite_float(c->high_hz) ||
      !finite_float(c->peak) || !finite_float(c->sample_hz) ||
      c->sample_hz < SWEEP_MIN_SAMPLE_HZ || c->sample_hz > SWEEP_MAX_SAMPLE_HZ ||
      c->sample_hz != (float)(unsigned)c->sample_hz ||
      c->low_hz < SWEEP_MIN_HZ || c->high_hz <= c->low_hz ||
      c->high_hz > SWEEP_MAX_HZ || c->high_hz > c->sample_hz * (1.0f / SWEEP_MIN_SAMPLES_PER_CYCLE) ||
      c->peak < SWEEP_MIN_PEAK_A || c->peak > SWEEP_MAX_PEAK_A ||
      (c->low_hz > SWEEP_BASELINE_HIGH_HZ && c->count < SWEEP_BASELINE_POINTS + 2u)) return 2;
  if (c->method == SWEEP_STEPPED) {
    unsigned anchor=c->low_hz>SWEEP_BASELINE_HIGH_HZ?SWEEP_BASELINE_POINTS:0u;
    unsigned first=(unsigned)(c->sample_hz/c->low_hz+0.5f);
    unsigned last=(unsigned)(c->sample_hz/c->high_hz+0.5f);
    for(i=0;i<c->count;++i) {
      unsigned p=c->periods[i];
      if(p<SWEEP_MIN_SAMPLES_PER_CYCLE || (float)p<c->sample_hz/SWEEP_MAX_HZ ||
         p>(unsigned)(c->sample_hz/SWEEP_MIN_HZ+0.5f) ||
         (i && p>=c->periods[i-1])) return 2;
      if(i<anchor) {
        if(p!=(unsigned)(c->sample_hz/(i?SWEEP_BASELINE_HIGH_HZ:SWEEP_MIN_HZ)+0.5f)) return 2;
      } else if(p>first || p<last) return 2;
    }
    if(c->periods[anchor]!=first || c->periods[c->count-1]!=last) return 2;
  }

  memset(e,0,sizeof(*e));
  e->method=(uint8_t)c->method; e->count=(uint8_t)c->count;
  e->sample_hz=c->sample_hz;
  e->length=(unsigned)c->sample_hz * SWEEP_CHIRP_PERIOD_MS / 1000u;
  if(c->method==SWEEP_STEPPED) {
    for(i=0;i<c->count;++i) {
      unsigned p=c->periods[i];
      e->periods[i]=(uint16_t)p;
      e->point_cycles[i]=(uint8_t)stepped_cycles((unsigned)c->sample_hz,p);
    }
    e->length=e->periods[0]; e->scale=c->peak;
    e->chirp.ramp_gain=4294967296.0f/(float)e->length;
    return 0;
  }
  maximum=sweep_chirp_prepare(&e->chirp,c->low_hz>SWEEP_BASELINE_HIGH_HZ?SWEEP_MIN_HZ:c->low_hz,
                              c->high_hz,c->sample_hz,e->length);
  e->scale=c->peak/(maximum*SWEEP_CHIRP_PEAK_HEADROOM);
  return 0;
}

void sweep_engine_start(sweep_engine_t *e)
{ 
  if(e && e->state==SWEEP_IDLE && e->count>=SWEEP_MIN_POINTS) {
    e->state=SWEEP_RUNNING; 
  }
}

float sweep_engine_step(sweep_engine_t *e,float feedback)
{
  float x;
  if (!e || e->state!=SWEEP_RUNNING) return 0.0f;
  if (!finite_float(feedback)) { sweep_engine_abort(e,SWEEP_NONFINITE); return 0.0f; }
  if(e->method==SWEEP_STEPPED) {
    /* Exact integer-period phase reset; no device DFT, libm or sample buffer. */
    x=e->scale*sweep_sine_phase((uint32_t)((float)e->position*e->chirp.ramp_gain));
    ++e->samples;
    if(++e->position==e->length) {
      e->position=0;
      if(++e->cycles==e->point_cycles[e->progress]) {
        e->cycles=0;
        if(++e->progress==e->count) e->state=SWEEP_DONE;
        else {e->length=e->periods[e->progress];e->chirp.ramp_gain=4294967296.0f/(float)e->length;}
      }
    }
    return x;
  }
  x=e->scale*sweep_chirp_step(&e->chirp,e->length,e->position);
  ++e->samples;
  if (++e->position==e->length) {
    e->position=0; sweep_chirp_reset(&e->chirp);
    ++e->cycles; e->progress=(uint8_t)((e->cycles*e->count)/SWEEP_CHIRP_TOTAL_PERIODS);
    if(e->cycles==SWEEP_CHIRP_TOTAL_PERIODS) e->state=SWEEP_DONE;
  }
  return x;
}
void sweep_engine_abort(sweep_engine_t *e,unsigned flags)
{
  if(e && (e->state==SWEEP_RUNNING || e->state==SWEEP_DONE)) {
    e->flags=(uint8_t)(e->flags|flags); e->state=SWEEP_ABORTED;
  }
}
