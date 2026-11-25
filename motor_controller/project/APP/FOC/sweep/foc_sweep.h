#ifndef FOC_SWEEP_H
#define FOC_SWEEP_H

#include "motor.h"
#include "sweep_engine.h"

#ifndef FOC_SWEEP_ENABLED
#define FOC_SWEEP_ENABLED 1
#endif
/* Independent commissioning bounds, at the MOTOR SHAFT (not joint/gearbox).
 * Override at build time only after evaluating the actual mechanism. */
#ifndef FOC_SWEEP_MAX_TRAVEL_RAD
#define FOC_SWEEP_MAX_TRAVEL_RAD 0.5f
#endif
#ifndef FOC_SWEEP_MAX_SPEED_RAD_S
#define FOC_SWEEP_MAX_SPEED_RAD_S 5.0f
#endif
#ifndef FOC_SWEEP_MAX_TEMP_C
#define FOC_SWEEP_MAX_TEMP_C 60.0f
#endif

typedef struct {
  uint16_t run, flags, max_us;
  uint8_t state, method, count, progress;
  uint32_t sample_hz, samples;
  uint16_t interval_min_us, interval_max_us, interval_last_us;
  uint16_t sense_max_us, control_max_us, engine_max_us;
  uint32_t failure_sample;
  uint8_t timing_reason;
  uint8_t motion_reason, motion_index; /* 18 travel / 19 speed; 255 = continuous/parallel excitation */
  float motion_value, motion_limit, motion_hz;
} foc_sweep_status_t;

#if FOC_SWEEP_ENABLED
/* Foreground-only rejection detail; caller-owned, no retained diagnostic RAM. */
enum {
  SWEEP_REJECT_NONE, SWEEP_REJECT_BUSY, SWEEP_REJECT_MOTOR,
  SWEEP_REJECT_STATE, SWEEP_REJECT_MODE, SWEEP_REJECT_DISARMED,
  SWEEP_REJECT_CALIBRATION, SWEEP_REJECT_FAULT, SWEEP_REJECT_REFERENCE,
  SWEEP_REJECT_REQUEST, SWEEP_REJECT_CURRENT_LIMIT, SWEEP_REJECT_START_SPEED,
  SWEEP_REJECT_Q_LIMIT, SWEEP_REJECT_D_LIMIT, SWEEP_REJECT_Q_RAMP,
  SWEEP_REJECT_D_RAMP, SWEEP_REJECT_FREQUENCIES, SWEEP_REJECT_NONFINITE,
  SWEEP_REJECT_TRAVEL, SWEEP_REJECT_SPEED, SWEEP_REJECT_BOARD_TEMP,
  SWEEP_REJECT_MOTOR_TEMP
};
typedef struct { uint8_t reason; float value; } foc_sweep_rejection_t;
/* Foreground-only control. These functions internally serialize with the ISR. */
int foc_sweep_start(motor_handle_t *m, const sweep_config_t *cfg, uint16_t run);
int foc_sweep_start_checked(motor_handle_t *m, const sweep_config_t *cfg, uint16_t run,
                            foc_sweep_rejection_t *rejection);
int foc_sweep_start_recorded(motor_handle_t *m, const sweep_config_t *cfg, uint16_t run,
                             foc_sweep_rejection_t *rejection, bool record);
void foc_sweep_stop(motor_handle_t *m, unsigned reason);
void foc_sweep_status(foc_sweep_status_t *out);
int foc_sweep_result(uint16_t run, unsigned index, sweep_result_t *out);

/* A diagnostic build checks this once at the FOC dispatch boundary. Only the
 * adapter writes the pointer, publishing after foreground preparation + DMB. */
extern motor_handle_t * volatile foc_sweep_owner;
#define foc_sweep_is_active(m) (foc_sweep_owner == (m))
typedef void (*foc_sweep_current_fn)(motor_handle_t *, motor_runtime_t *, float);
void foc_sweep_run_isr(motor_handle_t *m, motor_runtime_t *rt, uint32_t start_ticks,
                       foc_sweep_current_fn current_step);
void foc_sweep_on_disarm(motor_handle_t *m);
#else
#define foc_sweep_is_active(m) false
#define foc_sweep_run_isr(m, rt, ticks, current_step) ((void)0)
#define foc_sweep_on_disarm(m) ((void)0)
#define foc_sweep_stop(m, reason) ((void)0)
#endif
#endif
