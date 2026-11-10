#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "datatypes.h"
#include "encoder_types.h"

/* Directed per-sample travel accumulates until this axis is next estimated. */
typedef struct {
  uint32_t pending_counts; ///< 未处理的计数
  uint32_t time_us;
  uint32_t velocity_time_us;
  float raw_velocity_rad_s;
  bool valid;
  bool velocity_valid;
#if FOC_ENCODER_USE_PLL
  uint16_t accepted_counts;
  int32_t shadow_count;
  uint8_t init_count;
  bool pll_rejected;
  float raw_angle;
  encoder_pll_t pll;
#endif
} encoder_observer_t;

/**
 * 编码器模块：单/双路绑定、观测算法、发布 motor_state 机械观测量。
 */

void encoder_init(void);

/* Single sampling entry, owned by the ADC ISR. Read the shaft every tick, the joint only on outer_tick.
 * Sample at least once per 16-bit board timer period (65.536 ms) to extend time. */
void encoder_update(motor_handle_t *m, bool outer_tick);

/* Consume the latest valid measurement, without SPI. Returns false for a duplicate
 * or missing sample. The first valid sample is the baseline; one sample alone gives zero speed. */
bool encoder_estimate_velocity(motor_handle_t *m);

bool encoder_get_shaft_debug(const motor_handle_t *m, encoder_shaft_debug_t *dbg);

#endif /* __ENCODER_H__ */
