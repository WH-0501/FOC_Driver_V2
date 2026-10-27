/*
 * GD32 板级实现。工程中编入本文件并定义 PLATFORM_GD32，勿同时编入其它 board_*.c。
 *
 * GD32 外设 API 与 STM32 高度相似；假定底层已在 main 中完成初始化。
 * 预驱引脚通过 BOARD_GATE_* 宏配置（默认同 STM32 模板引脚）。
 */
#include "board.h"
#include "gd32f4xx_gpio.h"

#ifndef BOARD_GATE_NSLEEP_PORT
#define BOARD_GATE_NSLEEP_PORT GPIOA
#define BOARD_GATE_NSLEEP_PIN GPIO_PIN_15
#define BOARD_GATE_ENA_PORT GPIOA
#define BOARD_GATE_ENA_PIN GPIO_PIN_12
#define BOARD_GATE_ENB_PORT GPIOA
#define BOARD_GATE_ENB_PIN GPIO_PIN_11
#define BOARD_GATE_ENC_PORT GPIOB
#define BOARD_GATE_ENC_PIN GPIO_PIN_10
#define BOARD_GATE_FAULT_PORT GPIOB
#define BOARD_GATE_FAULT_PIN GPIO_PIN_3
#endif

/* 由 GD32 工程提供：相电流 ADC、PWM 定时器、微秒时基定时器句柄/寄存器 */
#ifndef GD32_PHASE_ADC
#error "GD32 port: define GD32_PHASE_ADC and implement injected ADC read in board_get_phase_current."
#endif

#ifndef GD32_PWM_TIMER
#error "GD32 port: define GD32_PWM_TIMER and PWM channel macros for set_pwm."
#endif

#ifndef GD32_TIME_US_TIMER
#error "GD32 port: define GD32_TIME_US_TIMER for board_time_us."
#endif

uint16_t pwm_compare_top = 0;

void board_init(void)
{
  /* GD32 工程在 main 中完成外设初始化后，此处仅缓存 PWM 周期 */
}

void board_deinit(void)
{
  (void)current_hw_deinit();
  (void)pwm_hw_stop();
}

uint32_t board_time_us(void)
{
  return timer_counter_read(GD32_TIME_US_TIMER);
}

int board_get_gate_hw(gate_hw_binding_t *hw)
{
  if (hw == NULL)
  {
    return -1;
  }

  hw->port_nSLEEP = BOARD_GATE_NSLEEP_PORT;
  hw->pin_nSLEEP = BOARD_GATE_NSLEEP_PIN;
  hw->port_ENA = BOARD_GATE_ENA_PORT;
  hw->pin_ENA = BOARD_GATE_ENA_PIN;
  hw->port_ENB = BOARD_GATE_ENB_PORT;
  hw->pin_ENB = BOARD_GATE_ENB_PIN;
  hw->port_ENC = BOARD_GATE_ENC_PORT;
  hw->pin_ENC = BOARD_GATE_ENC_PIN;
  hw->port_FAULT = BOARD_GATE_FAULT_PORT;
  hw->pin_FAULT = BOARD_GATE_FAULT_PIN;
  return 0;
}


/* This port has no qualified fresh-sequence polling implementation yet.
 * Fail explicitly rather than reporting calibration of repeated registers. */
error_t current_hw_calibration_begin(uint32_t *saved_state)
{
  (void)saved_state;
  return ERR_NOT_IMPL;
}
error_t current_hw_calibration_read(motor_handle_t *m)
{
  (void)m;
  return ERR_NOT_IMPL;
}
void current_hw_calibration_end(uint32_t saved_state) { (void)saved_state; }

error_t current_hw_init(void)
{
  /* 由 GD32 工程在移植时对接注入 ADC 中断启动 */
  return ERR_NONE;
}

error_t current_hw_deinit(void)
{
  return ERR_NONE;
}

/* These ports retain their previous modulation range. Their ADC timing
 * must be qualified in the target-specific project before low-side use. */
float pwm_hw_max_duty(void)
{
  return 1.0f;
}

error_t pwm_hw_init(void)
{
  return ERR_NONE;
}

error_t pwm_hw_start(void)
{
  return ERR_NONE;
}

error_t pwm_hw_lowside_brake_off(void)
{
  return ERR_NONE;
}

error_t pwm_hw_stop(void)
{
  (void)pwm_hw_lowside_brake_off();
  return ERR_NONE;
}

void board_fault_emergency_stop_hw(void)
{
  (void)pwm_hw_stop();
  gpio_bit_reset(BOARD_GATE_ENA_PORT, BOARD_GATE_ENA_PIN);
  gpio_bit_reset(BOARD_GATE_ENB_PORT, BOARD_GATE_ENB_PIN);
  gpio_bit_reset(BOARD_GATE_ENC_PORT, BOARD_GATE_ENC_PIN);
}

error_t pwm_hw_lowside_brake_on(void)
{
  return ERR_NONE;
}

error_t set_pwm(motor_actuation_t *actuation)
{
  (void)actuation;
  return ERR_NONE;
}

void board_get_phase_current(motor_handle_t *m)
{
  (void)m;
}

void board_current_loop_irq_handler(void *adc_handle)
{
  (void)adc_handle;
  board_invoke_current_loop();
}

void board_uart_stream_mode_set(uint8_t enabled)
{
  (void)enabled;
}

uint8_t board_uart_tx_try(const uint8_t *data, uint16_t len)
{
  (void)data;
  (void)len;
  return 0u;
}

uint8_t board_uart_log_try(const uint8_t *data, uint16_t len)
{
  (void)data;
  (void)len;
  return 0u;
}

uint8_t board_uart_tx_busy(void)
{
  return 0u;
}

void board_uart_dma_irq_handler(void)
{
}

void board_uart_get_diag(board_uart_diag_t *diag)
{
  if (diag == NULL)
  {
    return;
  }

  diag->ring_size = 0u;
  diag->used = 0u;
  diag->free = 0u;
  diag->tx_inflight = 0u;
  diag->dma_busy = 0u;
  diag->dropped_messages = 0u;
  diag->dropped_bytes = 0u;
}
