#include "sweep_engine.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846
static void model(unsigned method, float fs, float peak)
{
  sweep_engine_t e = {0};
  sweep_config_t cfg = {method, 16u, 20.0f, 2000.0f, peak, fs};
  double a = exp(-2.0 * PI * 800.0 / fs), y = 0.0, max_db = 0.0, max_phase = 0.0;
  unsigned n = 0u, i;
  assert(!sweep_engine_prepare(&e, &cfg));
  assert(e.state == SWEEP_IDLE);
  sweep_engine_start(&e);
  assert(sweep_engine_prepare(&e, &cfg) == 1);
  while (e.state == SWEEP_RUNNING) {
    float x = sweep_engine_step(&e, (float)y + 0.04f);
    assert(fabsf(x) <= peak * 1.0005f);
    y = a * y + (1.0 - a) * x;
    assert(++n < 500000u);
  }
  assert(e.state == SWEEP_DONE && e.flags == 0 && e.progress == cfg.count);
  if (method == SWEEP_MULTISINE) assert(n == (unsigned)(fs * 0.2f + 0.5f));
  if (method == SWEEP_CHIRP) assert(n == 4u*(unsigned)(fs+0.5f));
  for (i = 0u; i < cfg.count; ++i) {
    sweep_result_t r;
    double w, den, re, im, db, phase;
    assert(!sweep_engine_result(&e, i, &r));
    assert(r.blocks == (method != SWEEP_STEPPED_SINE ? 3u : 4u));
    w = 2.0 * PI * r.hz / fs;
    den = 1.0 - 2.0 * a * cos(w) + a * a;
    re = (1.0 - a) * (cos(w) - a) / den;
    im = -(1.0 - a) * sin(w) / den;
    db = fabs(20.0 * log10(hypot(r.real, r.imag) / hypot(re, im)));
    phase = fabs(atan2(r.imag, r.real) - atan2(im, re));
    if (db > max_db) max_db = db;
    if (phase > max_phase) max_phase = phase;
    assert(db < 0.10 && phase < 0.01 && r.coherence > 0.99f);
  }
  printf("PASS model method=%u fs=%.0f peak=%.3f samples=%u max_error=%.6f dB / %.6f rad\n",
         method, fs, peak, n, max_db, max_phase);
  /* A malformed retry must not destroy a completed measurement. */
  cfg.high_hz = NAN;
  assert(sweep_engine_prepare(&e, &cfg) == 2 && e.state == SWEEP_DONE);
  assert(sweep_engine_result(&e, cfg.count, &(sweep_result_t){0}) == 1);
  assert(sweep_engine_step(&e, 1.0f) == 0.0f);
}
static void chirp_regression(void)
{
  sweep_engine_t e = {0};
  sweep_config_t c = {SWEEP_CHIRP, 16, 600, 1600, .05f, 20000};
  sweep_result_t r;
  unsigned i, block;
  float reference[40000]; /* tests only: firmware never buffers raw samples */
  double sum;
  assert(!sweep_engine_prepare(&e, &c));
  assert(sweep_engine_frequency(&e,0)==20 && sweep_engine_frequency(&e,1)==40);
  assert(sweep_engine_frequency(&e,2)==600 && sweep_engine_frequency(&e,15)==1600);
  sweep_engine_start(&e);
  for (block=0; block<4; ++block) {
    sum=0;
    for (i=0; i<20000; ++i) {
      float hz=sweep_chirp_frequency(&e.chirp,c.sample_hz);
      float x=sweep_engine_step(&e,0);
      double expected=20.0+(1600.0-20.0)*i/19999.0;
      assert(fabs(hz-expected)<.001);
      assert(fabs(x)<=c.peak && isfinite(x));
      if (!block) reference[i]=x; else assert(x==reference[i]);
      if (i==0 || i==19999) assert(x==0);
      sum+=x;
    }
    assert(fabs(sum/20000)<1e-8); /* no sustained DC torque from excitation */
  }
  assert(e.state==SWEEP_DONE && e.samples==80000);
  assert(!sweep_engine_result(&e,0,&r) && r.coherence==0);
  /* Different cycle response must not report perfect coherence. */
  assert(!sweep_engine_prepare(&e,&c));sweep_engine_start(&e);
  while(e.state==SWEEP_RUNNING) {
    unsigned n=e.samples%20000u;
    float y=reference[n] * (e.blocks==1u ? -1.0f : 1.0f);
    sweep_engine_step(&e,y);
  }
  for(i=0;i<e.count;++i){assert(!sweep_engine_result(&e,i,&r));assert(r.coherence<.12f);}
  /* Exact coherent grid, duplicates rejected without clobbering prior result. */
  c.low_hz=850;c.high_hz=1100;c.count=16;
  assert(!sweep_engine_prepare(&e,&c));
  for(i=1;i<e.count;++i)assert(sweep_engine_frequency(&e,i)>sweep_engine_frequency(&e,i-1));
  c.low_hz=20;c.high_hz=21;
  assert(sweep_engine_prepare(&e,&c)==2);
  c.method=4;c.high_hz=2000;assert(sweep_engine_prepare(&e,&c)==2);
  /* 10 kHz is a supported numerical sample rate; upper bound remains Fs/10. */
  c.method=SWEEP_CHIRP;c.sample_hz=10000;c.high_hz=1000;
  assert(!sweep_engine_prepare(&e,&c));sweep_engine_start(&e);
  while(e.state==SWEEP_RUNNING)sweep_engine_step(&e,0);
  assert(e.samples==40000);
  puts("PASS chirp: exact repeats, frequency law, peak, zero mean/edges, baseline, grid, low coherence, 10 kHz");
}
int main(void)
{
  sweep_engine_t e = {0}, other = {0};
  sweep_config_t c = {SWEEP_MULTISINE, 16u, 20, 2000, 0.1f, 20000};
  sweep_result_t r;
  unsigned i;
  printf("RAM: engine=%u tone=%u bytes\n", (unsigned)sizeof(e), (unsigned)sizeof(e.tone[0]));
  assert(sizeof(e) <= 828 && sizeof(e.tone[0]) <= 48);
  model(1u, 20000, .1f); model(2u, 20000, .1f);
  model(3u, 20000, .05f); model(3u, 40000, .005f);
  model(1u, 40000, .005f); model(2u, 40000, .5f);
  assert(sweep_engine_prepare(NULL, &c) == 2 && sweep_engine_prepare(&e, NULL) == 2);
  c.peak = INFINITY; assert(sweep_engine_prepare(&e, &c) == 2);
  c.peak = -.1f; assert(sweep_engine_prepare(&e, &c) == 2);
  c.peak = .1f; c.sample_hz = 10000; assert(sweep_engine_prepare(&e, &c) == 2);
  c.sample_hz = 20000; c.count = 17; assert(sweep_engine_prepare(&e, &c) == 2);
  c.count = 16; c.high_hz = 40; assert(sweep_engine_prepare(&e, &c) == 2);
  c.low_hz = 700; c.high_hz = 1000; c.count = 3; assert(sweep_engine_prepare(&e, &c) == 2);
  c.count = 8; assert(!sweep_engine_prepare(&e, &c));
  assert(sweep_engine_frequency(&e, 0) == 20 && sweep_engine_frequency(&e, 1) == 40);
  for (i = 1u; i < c.count; ++i) assert(sweep_engine_frequency(&e, i) > sweep_engine_frequency(&e, i-1));
  sweep_engine_start(&e); assert(!sweep_engine_prepare(&other, &c));
  sweep_engine_start(&other);
  assert(sweep_engine_step(&e, NAN) == 0 && e.state == SWEEP_ABORTED);
  assert(other.state == SWEEP_RUNNING && other.samples == 0);
  sweep_engine_abort(&other, SWEEP_STOP);
  assert(other.flags == SWEEP_STOP && sweep_engine_result(&other, 0, &r) == 1);
  /* No response is retained as low quality, not fabricated as unity gain. */
  assert(!sweep_engine_prepare(&e, &c)); sweep_engine_start(&e);
  while (e.state == SWEEP_RUNNING) sweep_engine_step(&e, 0.0f);
  assert(!sweep_engine_result(&e, 0, &r) && r.coherence == 0 && r.output == 0);
  /* Independently changing phase each spectral block cannot have unity coherence. */
  c.method = SWEEP_STEPPED_SINE;
  assert(!sweep_engine_prepare(&e, &c)); sweep_engine_start(&e);
  while (e.state == SWEEP_RUNNING) {
    float y = .1f * ((e.blocks & 1u) ? -1.0f : 1.0f) * e.tone[e.index].s;
    sweep_engine_step(&e, y);
  }
  assert(!sweep_engine_result(&e, 0, &r) && r.coherence < .01f);
  chirp_regression();
  puts("PASS validation, quantization, retained results, context isolation, abort, zero response, coherence");
  return 0;
}
