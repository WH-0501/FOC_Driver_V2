#ifndef __BOARD_H__
#define __BOARD_H__

/**
 * 板级 BSP：电流采样 / PWM / UART / 微秒时基 / 预驱 GPIO 绑定。
 *
 * 平台选择（编译期，工程宏 + 链接对应 board_*.c，勿同时编入多个平台文件）：
 *   PLATFORM_AT32  -> board_at32.c
 *   PLATFORM_STM32 -> board_stm32.c
 *   PLATFORM_GD32  -> board_gd32.c
 *
 * board.c 仅承载电流环回调分发；其余平台相关实现均在 board_<platform>.c。
 */

#if !defined(PLATFORM_AT32) && !defined(PLATFORM_STM32) && !defined(PLATFORM_GD32)
#error "board.h: define PLATFORM_AT32, PLATFORM_STM32, or PLATFORM_GD32."
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "datatypes.h"
#include "error_types.h"
#include "gate_hw_binding.h"

typedef void (*board_current_loop_cb_t)(void);

/** 
 * @brief 板级初始化（如缓存 PWM 周期）；WorkBench/CubeMX 外设初始化仍在 main 中完成
 */
void board_init(void);
void board_deinit(void);

void board_current_loop_start(void);
void board_current_loop_stop(void);

int board_get_gate_hw(uint8_t axis_id, gate_hw_binding_t *hw);

void board_register_current_loop_callback(board_current_loop_cb_t cb);
void board_invoke_current_loop(void);

void board_get_phase_current(motor_handle_t *m);

void board_current_loop_irq_handler(void *adc_handle);
#define CURRENT_LOOP_IRQ_HANDLER board_current_loop_irq_handler

/** 自由运行微秒计数器快照（用于速度估算等）；须 board_init 后有效 */
uint32_t board_time_us(void);

error_t current_hw_init(void);
error_t current_hw_deinit(void);

error_t pwm_hw_init(uint8_t axis_id);
error_t pwm_hw_start(uint8_t axis_id);
error_t pwm_hw_stop(uint8_t axis_id);
error_t pwm_hw_lowside_brake_on(uint8_t axis_id);
error_t pwm_hw_lowside_brake_off(uint8_t axis_id);

error_t set_pwm(uint8_t axis_id, motor_actuation_t *actuation);

typedef struct
{
  uint16_t ring_size;
  uint16_t used;
  uint16_t free;
  uint16_t tx_inflight;
  uint8_t dma_busy;
  uint32_t dropped_messages;
  uint32_t dropped_bytes;
} board_uart_diag_t;

void board_uart_stream_mode_set(uint8_t enabled);
uint8_t board_uart_tx_try(const uint8_t *data, uint16_t len);
uint8_t board_uart_log_try(const uint8_t *data, uint16_t len);
uint8_t board_uart_tx_busy(void);
void board_uart_dma_irq_handler(void);
void board_uart_get_diag(board_uart_diag_t *diag);

#endif /* __BOARD_H__ */
