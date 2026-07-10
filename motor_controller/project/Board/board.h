#ifndef __BOARD_H__
#define __BOARD_H__

#if !defined(PLATFORM_STM32) && !defined(PLATFORM_AT32)
#error "board.h: define PLATFORM_STM32 or PLATFORM_AT32 in project preprocessor symbols."
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "datatypes.h"
#include "error_types.h"
#include "gate_hw_binding.h"

typedef void (*board_current_loop_cb_t)(void);

/** BSP：ADC/PWM/UART 等平台抽象（不开启电流环 IRQ，见 board_current_loop_start） */
void board_init(void);
void board_deinit(void);

/** 电流环 ISR 就绪：先 register_callback，再 start */
void board_current_loop_start(void);
void board_current_loop_stop(void);

/** 读取本板预驱 GPIO 绑定；成功返回 0 */
int board_get_gate_hw(gate_hw_binding_t *hw);

void board_register_current_loop_callback(board_current_loop_cb_t cb);
void board_invoke_current_loop(void);

void board_get_phase_current(motor_handle_t *m);

void board_current_loop_irq_handler(void *adc_handle);
#define CURRENT_LOOP_IRQ_HANDLER board_current_loop_irq_handler

error_t current_hw_init(void);
error_t current_hw_deinit(void);

error_t pwm_hw_init(void);
error_t pwm_hw_start(void);
error_t pwm_hw_stop(void);
error_t pwm_hw_lowside_brake_on(void);
error_t pwm_hw_lowside_brake_off(void);

error_t set_pwm(motor_actuation_t *actuation);

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
