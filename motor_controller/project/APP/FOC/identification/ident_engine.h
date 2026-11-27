#ifndef IDENT_ENGINE_H
#define IDENT_ENGINE_H
#include <stdbool.h>
#include <stdint.h>

/* All currents are phase-peak dq quantities; speed is SHAFT rad/s. */
typedef enum { IDENT_RSLS = 1, IDENT_FLUX = 2, IDENT_ALL = 3 } ident_mode_t;
typedef enum {
  IDENT_IDLE, IDENT_RS_LOW, IDENT_RS_HIGH, IDENT_LD, IDENT_LQ,
  IDENT_SPIN, IDENT_FLUX_SAMPLE, IDENT_DONE, IDENT_FAILED, IDENT_CANCELLED
} ident_stage_t;
typedef enum {
  IDENT_OK, IDENT_BAD_CONFIG, IDENT_CURRENT, IDENT_MOTION, IDENT_TIMEOUT,
  IDENT_SIGNAL, IDENT_NONFINITE, IDENT_VOLTAGE, IDENT_EXTERNAL, IDENT_TIMING
} ident_error_t;
typedef struct {
  float current_a;        /* high Rs current; flux torque current ceiling */
  float voltage_v;        /* absolute vector voltage ceiling */
  float injection_v;      /* AC peak, additionally capped by measured Rs */
  float injection_hz;
  float speed_rad_s;      /* positive shaft speed for flux */
  float current_trip_a;   /* instantaneous phase/dq magnitude trip */
  float speed_trip_rad_s;
} ident_config_t;
typedef struct {
  float rs, ld, lq, flux;
  uint8_t valid;          /* bit0=Rs/Ld/Lq, bit1=flux; only valid on DONE */
} ident_result_t;
typedef struct {
  float id, iq, speed, position, phase_peak;
  float vd, vq;          /* reconstructed APPLIED voltage in rotor dq frame */
} ident_sample_t;
typedef struct { float vd, vq; } ident_output_t;
typedef struct {
  ident_config_t cfg;
  ident_result_t result;
  ident_stage_t stage;
  ident_error_t error;
  ident_mode_t mode;
  uint32_t tick, total, period, n;
  float dt, pole_pairs, origin, vd_int, vq_int;
  float sum_i, sum_v, sum_i2, sum_v2, sum_s, sum_c, sum_vs, sum_vc;
  float low_i, low_v, amplitude, first_iq, flux_num, flux_den;
  float min_speed, max_speed;
} ident_engine_t;
bool ident_engine_start(ident_engine_t *e, const ident_config_t *cfg, ident_mode_t mode,
                        float dt, unsigned pole_pairs, const ident_result_t *seed,
                        float position);
void ident_engine_abort(ident_engine_t *e, ident_error_t error);
bool ident_engine_active(const ident_engine_t *e);
ident_output_t ident_engine_step(ident_engine_t *e, const ident_sample_t *s);
#endif
