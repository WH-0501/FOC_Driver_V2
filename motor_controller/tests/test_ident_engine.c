#include "ident_engine.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static ident_config_t config(void) {
  ident_config_t c = {0.3f, 3.0f, 0.2f, 200.0f, 20.0f, 1.0f, 30.0f}; return c;
}
static int close_to(float x, float y, float relative) { return fabsf(x-y)<relative*y; }
static void simulate(float r, float ld, float lq, float flux, ident_mode_t mode, float delay) {
  ident_engine_t e;
  ident_config_t c = config();
  ident_result_t seed = {r,ld,lq,0,0};
  ident_sample_t s = {0};
  ident_output_t u = {0}, previous = {0};
  float id=0,iq=0,w=0,pos=0;
  const float dt=0.00005f;
  assert(ident_engine_start(&e,&c,mode,dt,5,&seed,0));
  for (unsigned k=0;k<600001 && ident_engine_active(&e);k++) {
    /* Independent continuous dq plant integrated at 500 kHz, with a fractional
     * PWM update delay, saliency, mechanical inertia and viscous load. */
    for (unsigned j=0;j<25;j++) {
      float h=dt/25, we=5*w;
      ident_output_t applied = (j < (unsigned)(25*delay)) ? previous : u;
      float did=(applied.vd-r*id+we*lq*iq)/ld;
      float diq=(applied.vq-r*iq-we*(flux+ld*id))/lq;
      float dw=(7.5f*(flux*iq+(ld-lq)*id*iq)-0.0002f*w)/0.0003f;
      id+=did*h; iq+=diq*h; w+=dw*h; pos+=w*h;
    }
    s.id=id; s.iq=iq; s.speed=w; s.position=pos;
    s.phase_peak=sqrtf(id*id+iq*iq); s.vd=u.vd; s.vq=u.vq;
    previous=u; u=ident_engine_step(&e,&s);
  }
  printf("mode=%d stage=%d error=%d R=%g Ld=%g Lq=%g flux=%g\n",mode,e.stage,e.error,
          e.result.rs,e.result.ld,e.result.lq,e.result.flux);
  fflush(stdout);
  assert(e.stage==IDENT_DONE && e.error==IDENT_OK);
  if (mode != IDENT_FLUX) {
    assert(close_to(e.result.rs,r,0.03f));
    assert(close_to(e.result.ld,ld,0.06f)); assert(close_to(e.result.lq,lq,0.06f));
  }
  if (mode != IDENT_RSLS) assert(close_to(e.result.flux,flux,0.04f));
  assert(e.result.valid==(mode==IDENT_ALL?3:mode==IDENT_FLUX?2:1));
  assert(u.vd==0 && u.vq==0);
}
static void guards(void) {
  ident_engine_t e; ident_config_t c=config(); ident_sample_t s={0}; ident_output_t u;
  assert(!ident_engine_start(&e,&c,IDENT_FLUX,0.00005f,5,0,0));
  c.current_a=NAN; assert(!ident_engine_start(&e,&c,IDENT_RSLS,0.00005f,5,0,0)); c=config();
  c.injection_hz=20000; assert(!ident_engine_start(&e,&c,IDENT_RSLS,0.00005f,5,0,0)); c=config();
  assert(ident_engine_start(&e,&c,IDENT_RSLS,0.00005f,5,0,0));
  s.id=NAN; u=ident_engine_step(&e,&s); assert(e.error==IDENT_NONFINITE && u.vd==0);
  s.id=0; assert(ident_engine_start(&e,&c,IDENT_RSLS,0.00005f,5,0,0));
  s.phase_peak=1; u=ident_engine_step(&e,&s); assert(e.error==IDENT_CURRENT && u.vq==0);
  s.phase_peak=0; s.position=0.03f;
  assert(ident_engine_start(&e,&c,IDENT_RSLS,0.00005f,5,0,0));
  ident_engine_step(&e,&s); assert(e.error==IDENT_MOTION);
  s.position=0;
  assert(ident_engine_start(&e,&c,IDENT_RSLS,0.00005f,5,0,0));
  for(unsigned i=0;i<25000 && ident_engine_active(&e);i++) ident_engine_step(&e,&s);
  assert(e.stage==IDENT_FAILED && e.error==IDENT_SIGNAL && e.result.valid==0);
  assert(ident_engine_start(&e,&c,IDENT_ALL,0.00005f,5,0,0));
  ident_engine_abort(&e,IDENT_OK); assert(e.stage==IDENT_CANCELLED && e.result.valid==0);
  u=ident_engine_step(&e,&s); assert(u.vd==0 && u.vq==0);
  /* A locked rotor must never produce a flux result. */
  ident_result_t seed={1.5f,0.0012f,0.0018f,0,0};
  assert(ident_engine_start(&e,&c,IDENT_FLUX,0.00005f,5,&seed,0));
  for(unsigned i=0;i<200000 && ident_engine_active(&e);i++) ident_engine_step(&e,&s);
  assert(e.error==IDENT_TIMEOUT && e.result.valid==0);
}
int main(void) {
  guards();
  simulate(1.5f,0.0012f,0.0018f,0.015f,IDENT_ALL,0);
  simulate(0.3f,0.0003f,0.0005f,0.01f,IDENT_RSLS,0.4f);
  simulate(1.5f,0.0012f,0.0018f,0.015f,IDENT_FLUX,0.4f);
  puts("PASS identification numerical model and guards"); return 0;
}
