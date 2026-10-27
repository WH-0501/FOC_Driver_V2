/*
 * STM32 板级实现。工程中编入本文件并定义 PLATFORM_STM32，勿同时编入其它 board_*.c。
 *
 * 假定 CubeMX/HAL 已在 main 中完成时钟、GPIO、ADC2、TIM1、TIM6 初始化；
 * board_init() 仅做电机功率级就绪检查；电流环/PWM 运行时控制与 AT32 语义对齐。
 *
 * 预驱引脚：在 main.h 或工程宏中定义 BOARD_GATE_*（见下方 #ifndef 默认值）。
 */
#include "board.h"
#include "main.h"

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

#ifndef HADC_PHASE_CURRENT
extern ADC_HandleTypeDef hadc2;
#define HADC_PHASE_CURRENT (&hadc2)
#endif

#ifndef PWM_TIM_HANDLE
extern TIM_HandleTypeDef htim1;
#define PWM_TIM_HANDLE (&htim1)
#endif

#ifndef BOARD_TIME_US_HANDLE
extern TIM_HandleTypeDef htim6;
#define BOARD_TIME_US_HANDLE (&htim6)
#endif

#define PWM_TIME_U_CHANNEL TIM_CHANNEL_1
#define PWM_TIME_V_CHANNEL TIM_CHANNEL_2
#define PWM_TIME_W_CHANNEL TIM_CHANNEL_3

uint16_t pwm_compare_top = 0;

static uint16_t pwm_get_compare_top(void)
{
  return (uint16_t)PWM_TIM_HANDLE->Instance->ARR;
}

void board_init(void)
{
  pwm_compare_top = pwm_get_compare_top();
}

void board_deinit(void)
{
  (void)current_hw_deinit();
  (void)pwm_hw_stop();
}

uint32_t board_time_us(void)
{
  return (uint32_t)__HAL_TIM_GET_COUNTER(BOARD_TIME_US_HANDLE);
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
  if (HAL_ADCEx_Calibration_Start(HADC_PHASE_CURRENT, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return ERR_FAIL;
  }
  if (HAL_ADCEx_InjectedStart_IT(HADC_PHASE_CURRENT) != HAL_OK)
  {
    return ERR_FAIL;
  }
  return ERR_NONE;
}

error_t current_hw_deinit(void)
{
  if (HAL_ADCEx_InjectedStop_IT(HADC_PHASE_CURRENT) != HAL_OK)
  {
    return ERR_FAIL;
  }
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
  if (pwm_compare_top == 0U)
  {
    pwm_compare_top = pwm_get_compare_top();
  }

  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, 0U);
  return ERR_NONE;
}

error_t pwm_hw_start(void)
{
  if (HAL_TIM_PWM_Start(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL) != HAL_OK)
  {
    return ERR_FAIL;
  }
  if (HAL_TIM_PWM_Start(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL) != HAL_OK)
  {
    return ERR_FAIL;
  }
  if (HAL_TIM_PWM_Start(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL) != HAL_OK)
  {
    return ERR_FAIL;
  }
  return ERR_NONE;
}

error_t pwm_hw_lowside_brake_off(void)
{
  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, 0U);
  return ERR_NONE;
}

error_t pwm_hw_stop(void)
{
  (void)pwm_hw_lowside_brake_off();

  if (HAL_TIM_PWM_Stop(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL) != HAL_OK)
  {
    return ERR_FAIL;
  }
  if (HAL_TIM_PWM_Stop(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL) != HAL_OK)
  {
    return ERR_FAIL;
  }
  if (HAL_TIM_PWM_Stop(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL) != HAL_OK)
  {
    return ERR_FAIL;
  }
  return ERR_NONE;
}

void board_fault_emergency_stop_hw(void)
{
  (void)pwm_hw_stop();
  HAL_GPIO_WritePin(BOARD_GATE_ENA_PORT, BOARD_GATE_ENA_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BOARD_GATE_ENB_PORT, BOARD_GATE_ENB_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BOARD_GATE_ENC_PORT, BOARD_GATE_ENC_PIN, GPIO_PIN_RESET);
}

error_t pwm_hw_lowside_brake_on(void)
{
  if (pwm_compare_top == 0U)
  {
    pwm_compare_top = pwm_get_compare_top();
  }

  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, pwm_compare_top);
  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, pwm_compare_top);
  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, pwm_compare_top);
  return ERR_NONE;
}

error_t set_pwm(motor_actuation_t *actuation)
{
  if (actuation == NULL)
  {
    return ERR_FAIL;
  }

  if (pwm_compare_top == 0U)
  {
    pwm_compare_top = pwm_get_compare_top();
  }

  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL,
                        (uint32_t)(actuation->duty_a * (float)pwm_compare_top));
  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL,
                        (uint32_t)(actuation->duty_b * (float)pwm_compare_top));
  __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL,
                        (uint32_t)(actuation->duty_c * (float)pwm_compare_top));
  return ERR_NONE;
}

void board_get_phase_current(motor_handle_t *m)
{
  motor_state_t *s;

  if (m == NULL)
  {
    return;
  }

  s = &m->state;
  s->board_temp.adc_raw =
      (uint16_t)HAL_ADCEx_InjectedGetValue(HADC_PHASE_CURRENT, ADC_INJECTED_RANK_1);
  s->phase_current.adc_raw[0] =
      (uint16_t)HAL_ADCEx_InjectedGetValue(HADC_PHASE_CURRENT, ADC_INJECTED_RANK_2);
  s->phase_current.adc_raw[1] =
      (uint16_t)HAL_ADCEx_InjectedGetValue(HADC_PHASE_CURRENT, ADC_INJECTED_RANK_3);
  s->phase_current.adc_raw[2] =
      (uint16_t)HAL_ADCEx_InjectedGetValue(HADC_PHASE_CURRENT, ADC_INJECTED_RANK_4);
}

void board_current_loop_irq_handler(void *adc_handle)
{
  ADC_HandleTypeDef *adc_inst = (ADC_HandleTypeDef *)adc_handle;

  if ((adc_inst != NULL) && (adc_inst->Instance == ADC2))
  {
    board_invoke_current_loop();
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  board_current_loop_irq_handler((void *)hadc);
}

static volatile uint8_t s_board_uart_stream_mode;

void board_uart_stream_mode_set(uint8_t enabled)
{
  s_board_uart_stream_mode = (enabled != 0u) ? 1u : 0u;
}

uint8_t board_uart_tx_try(const uint8_t *data, uint16_t len)
{
  extern UART_HandleTypeDef huart1;

  if ((data == NULL) || (len == 0u))
  {
    return 0u;
  }
  return (HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 10u) == HAL_OK) ? 1u : 0u;
}

uint8_t board_uart_log_try(const uint8_t *data, uint16_t len)
{
  if (s_board_uart_stream_mode != 0u)
  {
    return 0u;
  }
  return board_uart_tx_try(data, len);
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
