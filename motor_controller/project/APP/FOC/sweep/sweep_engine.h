#ifndef FOC_SWEEP_ENGINE_H
#define FOC_SWEEP_ENGINE_H

#include <stdint.h>
#include "sweep_chirp.h"

#define SWEEP_MAX_POINTS 16u
enum { SWEEP_IDLE, SWEEP_RUNNING, SWEEP_DONE, SWEEP_ABORTED };
enum { SWEEP_MULTISINE = 1, SWEEP_STEPPED_SINE = 2, SWEEP_CHIRP = 3 };
enum {
  SWEEP_STOP = 1u, SWEEP_LIMIT = 2u, SWEEP_TIMING = 4u,
  SWEEP_COMMAND = 8u, SWEEP_MOTION = 16u, SWEEP_NONFINITE = 32u
};
/* Return codes: 0 success, 1 busy/unavailable, 2 invalid parameters/data. */
typedef struct {
  unsigned method, count;
  float low_hz, high_hz, peak, sample_hz;
} sweep_config_t;
typedef struct {
  float hz, input, output, real, imag, coherence;
  uint16_t blocks;
} sweep_result_t;
/* 48 bytes/tone: all signal/statistics fields remain binary32. X is calculated
 * once from the exact periodic reference; Y sums preserve multi-block H1/coherence.
 * Initial phases live in flash, and timing metadata is derived from bin/period. */
typedef struct {
  float s, c, ds, dc;
  float xr, xi;                 /* precomputed normalized reference spectrum */
  float yr, yi, sum_r, sum_i, yy;
  uint16_t bin_or_period, settle;      /* multisine bin, or stepped-sine sample period */
} sweep_tone_t;
/* Caller-owned, single writer; no hardware, heap, global context or shared RAM.
 * prepare/result are foreground operations; step/abort belong to the sample ISR. */
typedef struct {
  uint32_t samples;
  float sample_hz, scale;
  uint16_t remaining, length, block_remaining;
  uint8_t state, flags, method, count, index, progress, blocks;
  sweep_tone_t tone[SWEEP_MAX_POINTS];
  sweep_chirp_t chirp; /* used only by mode 3; other modes unchanged */
} sweep_engine_t;

float sweep_engine_frequency(const sweep_engine_t *e, unsigned index);

int sweep_engine_prepare(sweep_engine_t *e, const sweep_config_t *cfg);
void sweep_engine_start(sweep_engine_t *e);
/* y[n] is sampled BEFORE returned x[n] is applied. H includes that real delay.
 * Call exactly once per sample period. Last x is still a measurement sample;
 * the adapter must validate that sample before publishing completion. */
float sweep_engine_step(sweep_engine_t *e, float feedback);
void sweep_engine_abort(sweep_engine_t *e, unsigned flags);
int sweep_engine_result(const sweep_engine_t *e, unsigned index, sweep_result_t *out);

#endif
