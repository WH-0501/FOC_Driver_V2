#ifndef FOC_SWEEP_ENGINE_H
#define FOC_SWEEP_ENGINE_H
#include <stdint.h>
#include "sweep_config.h"
#include "sweep_chirp.h"
enum { SWEEP_IDLE, SWEEP_RUNNING, SWEEP_DONE, SWEEP_ABORTED };
/* Mode 2 requires an explicit AD/2 integer-period plan; mode 1 is retired. */
enum { SWEEP_STEPPED = 2, SWEEP_CHIRP = 3 };
enum { SWEEP_STOP=1u, SWEEP_LIMIT=2u, SWEEP_TIMING=4u, SWEEP_COMMAND=8u,
       SWEEP_MOTION=16u, SWEEP_NONFINITE=32u, SWEEP_RECORD_GAP=64u };
typedef struct {
  unsigned method;  ///< 扫频模式: 2逐频正弦/3Chirp；旧多正弦已停用
  unsigned count;   ///< 扫频点数. 最大值为16
  float low_hz;     ///< 扫频频率下限
  float high_hz;    ///< 扫频频率上限
  float peak;       ///< 扫频峰值 A（协议的 mA 在解码时转换）
  float sample_hz;  ///< 扫频采样率
  uint16_t periods[SWEEP_MAX_POINTS]; /* mode 2 only: actual Hz = fs / period */
} sweep_config_t;
/* Only excitation state. No device spectral estimator or sample history.
 * Device timing comes from sweep_config.h and is reported via AE/2.
 * The host discards settling periods and analyzes all measurement periods. */
typedef struct {
  uint32_t samples;
  float sample_hz, scale;
  uint32_t length, position;
  uint8_t state, flags, method, count, progress, cycles;
  sweep_chirp_t chirp;
  uint16_t periods[SWEEP_MAX_POINTS];
  uint8_t point_cycles[SWEEP_MAX_POINTS];
} sweep_engine_t;
int sweep_engine_prepare(sweep_engine_t *, const sweep_config_t *);
void sweep_engine_start(sweep_engine_t *);
/* Feedback is checked for finiteness, never filtered. y[n] precedes x[n]. */
float sweep_engine_step(sweep_engine_t *, float feedback);
void sweep_engine_abort(sweep_engine_t *, unsigned flags);
#endif
