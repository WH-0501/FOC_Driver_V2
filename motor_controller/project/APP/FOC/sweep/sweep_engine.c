#include "sweep_engine.h"
#include <float.h>
#include <string.h>
static int finite_float(float f) { return f >= -FLT_MAX && f <= FLT_MAX; }
int sweep_engine_prepare(sweep_engine_t *e, const sweep_config_t *c)
{
  float maximum;
  unsigned i;
  if (!e || !c) return 2;
  if (e->state == SWEEP_RUNNING) return 1;
  if ((c->method != SWEEP_CHIRP && c->method != SWEEP_STEPPED) || c->count < 3u || c->count > SWEEP_MAX_POINTS ||
      !finite_float(c->low_hz) || !finite_float(c->high_hz) ||
      !finite_float(c->peak) || !finite_float(c->sample_hz) ||
      c->sample_hz < 10000.0f || c->sample_hz > 40000.0f ||
      c->sample_hz != (float)(unsigned)c->sample_hz ||
      c->low_hz < 20.0f || c->high_hz <= c->low_hz ||
      c->high_hz > 2000.0f || c->high_hz > c->sample_hz * 0.1f ||
      c->peak < 0.005f || c->peak > 0.5f ||
      (c->low_hz > 40.0f && c->count < 4u)) return 2;
  if (c->method == SWEEP_STEPPED) {
    unsigned anchor=c->low_hz>40.0f?2u:0u;
    unsigned first=(unsigned)(c->sample_hz/c->low_hz+0.5f);
    unsigned last=(unsigned)(c->sample_hz/c->high_hz+0.5f);
    for(i=0;i<c->count;++i) {
      unsigned p=c->periods[i];
      if(p<10u || (float)p<c->sample_hz/2000.0f ||
         p>(unsigned)(c->sample_hz/20.0f+0.5f) ||
         (i && p>=c->periods[i-1])) return 2;
      if(i<anchor) {
        if(p!=(unsigned)(c->sample_hz/(i?40.0f:20.0f)+0.5f)) return 2;
      } else if(p>first || p<last) return 2;
    }
    if(c->periods[anchor]!=first || c->periods[c->count-1]!=last) return 2;
  }

  memset(e,0,sizeof(*e));
  e->method=(uint8_t)c->method; e->count=(uint8_t)c->count;
  // 在 20KHz 采样率下, 1s 采样点数为 20000点
  e->sample_hz=c->sample_hz; e->length=(uint16_t)c->sample_hz;
  if(c->method==SWEEP_STEPPED) {
    for(i=0;i<c->count;++i) {
      unsigned p=c->periods[i], settle=(((unsigned)c->sample_hz+49u)/50u+p-1u)/p;
      if(settle<4u) settle=4u;
      e->periods[i]=(uint16_t)p; e->point_cycles[i]=(uint8_t)(settle+16u);
    }
    e->length=e->periods[0]; e->scale=c->peak;
    e->chirp.ramp_gain=4294967296.0f/(float)e->length;
    return 0;
  }
  maximum=sweep_chirp_prepare(&e->chirp,c->low_hz>40.0f?20.0f:c->low_hz,
                              c->high_hz,c->sample_hz,e->length);
  e->scale=c->peak/(maximum*1.001f);
  return 0;
}

void sweep_engine_start(sweep_engine_t *e)
{ 
  if(e && e->state==SWEEP_IDLE && e->count>=3u) {
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
    ++e->cycles; e->progress=(uint8_t)((e->cycles*e->count)/4u);
    if(e->cycles==4u) e->state=SWEEP_DONE; // 固定4个完整1s周期
  }
  return x;
}
void sweep_engine_abort(sweep_engine_t *e,unsigned flags)
{
  if(e && (e->state==SWEEP_RUNNING || e->state==SWEEP_DONE)) {
    e->flags=(uint8_t)(e->flags|flags); e->state=SWEEP_ABORTED;
  }
}
