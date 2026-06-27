/*
 * STM32：工程中关闭 board_at32.c；可定义 HADC_PHASE_CURRENT（默认 &hadc2）。
 */
#include "board.h"
#include "foc.h"
#include "main.h"

// #define USE_6X_PWM_CTRL

#define TIM1_CLK_MHZ  (168)
#define PWM_FREQUENCY (20000)
#define PWM_PERIOD_CYCLES (uint16_t)((TIM1_CLK_MHZ * (uint32_t)1000000u / ((uint32_t)(PWM_FREQUENCY))) & 0xFFFE)
#define PWM_TIM_ARR (uint16_t)(PWM_PERIOD_CYCLES / 2u)  // 16位最大值为65535
// #define PWM_TIM_ARR  (6000)


extern ADC_HandleTypeDef hadc2;
#define HADC_PHASE_CURRENT (&hadc2)

#define PWM_TIM_HANDLE (&htim1)
#define PWM_TIME_U_CHANNEL TIM_CHANNEL_1
#define PWM_TIME_V_CHANNEL TIM_CHANNEL_2
#define PWM_TIME_W_CHANNEL TIM_CHANNEL_3
// #define SET_U_H_PWM(val)      (PWM_TIM_HANDLE.Instance->CCR1 = val)
// #define SET_V_H_PWM(val)      (PWM_TIM_HANDLE.Instance->CCR2 = val)
// #define SET_W_H_PWM(val)      (PWM_TIM_HANDLE.Instance->CCR3 = val)
// #define SET_U_L_PWM(val)      (val)
// #define SET_V_L_PWM(val)      (val)
// #define SET_W_L_PWM(val)      (val)
#define SET_U_H_PWM(val)      __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, value)
#define SET_V_H_PWM(val)      __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, value)
#define SET_W_H_PWM(val)      __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, value)

uint16_t pwm_compare_top = 0;

extern void foc_control_loop(void);

static uint16_t pwm_compare_top(void)
{
  return (uint16_t)TIM1->ARR;
}

error_t current_hw_init(void)
{
  // 初始化 ADC
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

error_t pwm_hw_init(void)
{
  // 初始化 PWM
  HAL_TIM_Base_Start_IT(PWM_TIM_HANDLE); /* 开启PWM的定时器周期中断 */
  HAL_TIM_OC_Start_IT(PWM_TIM_HANDLE, TIM_CHANNEL_4); /* 开启通道四的触发中断 */

  pwm_compare_top = pwm_compare_top();

  // 关闭 PWM 波形输出
  SET_U_H_PWM(0);
  SET_V_H_PWM(0);
  SET_W_H_PWM(0);
#ifdef USE_6X_PWM_CTRL
  SET_U_L_PWM(0);
  SET_V_L_PWM(0);
  SET_W_L_PWM(0);
#endif
  return ERR_NONE;
}

error_t pwm_hw_deinit(void)
{
  // 停止 PWM
  if (HAL_TIM_Base_Stop_IT(PWM_TIM_HANDLE) != HAL_OK)
  {
    return ERR_FAIL;
  }
  if (HAL_TIM_PWM_Stop_IT(PWM_TIM_HANDLE, TIM_CHANNEL_4) != HAL_OK)
  {
    return ERR_FAIL;
  }
  return ERR_NONE;
}

error_t pwm_hw_start(void)
{
  // 启动 PWM
  if (HAL_TIM_PWM_Start(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL) != HAL_OK
    #ifdef USE_6X_PWM_CTRL
      || HAL_TIMEx_PWMN_Start(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL) != HAL_OK
    #endif
    )
  {
    return ERR_FAIL;
  }
  if (HAL_TIM_PWM_Start(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL) != HAL_OK
    #ifdef USE_6X_PWM_CTRL
      || HAL_TIMEx_PWMN_Start(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL) != HAL_OK
    #endif
    )
  {
    return ERR_FAIL;
  }
  if (HAL_TIM_PWM_Start(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL) != HAL_OK
    #ifdef USE_6X_PWM_CTRL
      || HAL_TIMEx_PWMN_Start(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL) != HAL_OK
    #endif
    )
  {
    return ERR_FAIL;
  }
  return ERR_NONE;
}

error_t pwm_hw_stop(void)
{
  // 停止 PWM
  if (HAL_TIM_PWM_Stop(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL) != HAL_OK 
    #ifdef USE_6X_PWM_CTRL
      || HAL_TIMEx_PWMN_Stop(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL) != HAL_OK
    #endif
    )
  {
    return ERR_FAIL;
  }
  
  if (HAL_TIM_PWM_Stop(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL) != HAL_OK
    #ifdef USE_6X_PWM_CTRL
      || HAL_TIMEx_PWMN_Stop(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL) != HAL_OK
    #endif
    )
  {
    return ERR_FAIL;
  }

  if (HAL_TIM_PWM_Stop(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL) != HAL_OK
    #ifdef USE_6X_PWM_CTRL
      || HAL_TIMEx_PWMN_Stop(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL) != HAL_OK
    #endif
    )
  {
    return ERR_FAIL;
  }

  return ERR_NONE;
}

error_t set_pwm(motor_actuation_t *actuation)
{
  SET_U_H_PWM(actuation->duty_a * pwm_compare_top); // or use PWM_TIM_ARR replace pwm_compare_top
  SET_V_H_PWM(actuation->duty_b * pwm_compare_top);
  SET_W_H_PWM(actuation->duty_c * pwm_compare_top);
// #ifdef USE_6X_PWM_CTRL
//   SET_U_L_PWM(actuation->duty_a * PWM_TIM_ARR);
//   SET_V_L_PWM(actuation->duty_b * PWM_TIM_ARR);
//   SET_W_L_PWM(actuation->duty_c * PWM_TIM_ARR);
// #endif
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

  if (adc_inst->Instance == ADC2)
  {
    foc_control_loop();
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
