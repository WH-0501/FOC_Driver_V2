/*
 * AT32M416：工程中关闭 board_stm32.c。
 * ADC/TMR 外设与时基由 WorkBench（wk_adc*.c / wk_tmr.c）初始化；本文件负责电流环抢占采样中断开关、
 * PWM 三相通道启停与占空比写入，语义对齐 board_stm32.c。
 *
 * PWM 计数周期与 wk_tmr1_init() 中 tmr_base_init(TMR1, pr, ...) 的 pr 一致，占空比按 compare/pr 缩放。
 */
#include "board.h"
#include "at32m412_416_wk_config.h"
#include "at32m412_416_adc.h"
#include "at32m412_416_tmr.h"
#include "wk_dma.h"
#include "wk_usart.h"
#include <string.h>

#define PWM_TIM_HANDLE TMR1
#define PWM_TIME_U_CHANNEL TMR_SELECT_CHANNEL_1
#define PWM_TIME_V_CHANNEL TMR_SELECT_CHANNEL_2
#define PWM_TIME_W_CHANNEL TMR_SELECT_CHANNEL_3

uint16_t pwm_compare_top = 0;

#ifndef BOARD_UART_RAW_TX_BUF_SIZE
#define BOARD_UART_RAW_TX_BUF_SIZE 256u
#endif

static uint8_t s_board_uart_dma_inited;
static volatile uint8_t s_board_uart_stream_mode;
static uint8_t s_board_uart_raw_buf[BOARD_UART_RAW_TX_BUF_SIZE];
static volatile uint8_t s_board_uart_busy;
static volatile uint16_t s_board_uart_tx_len;
static volatile uint32_t s_board_uart_dropped_messages;
static volatile uint32_t s_board_uart_dropped_bytes;

static uint16_t pwm_get_compare_top(void)
{
  return (uint16_t)PWM_TIM_HANDLE->pr;
}

int board_get_gate_hw(gate_hw_binding_t *hw)
{
  if (hw == NULL)
  {
    return -1;
  }

  hw->port_nSLEEP = nSLEEP_GPIO_PORT;
  hw->pin_nSLEEP = nSLEEP_PIN;
  hw->port_ENA = ENA_GPIO_PORT;
  hw->pin_ENA = ENA_PIN;
  hw->port_ENB = ENB_GPIO_PORT;
  hw->pin_ENB = ENB_PIN;
  hw->port_ENC = ENC_GPIO_PORT;
  hw->pin_ENC = ENC_PIN;
  hw->port_FAULT = nFAULT_GPIO_PORT;
  hw->pin_FAULT = nFAULT_PIN;
  return 0;
}

error_t current_hw_init(void)
{
  /* wk_adc2_init() 已完成校准与硬件触发绑定；此处打开抢占转换结束中断，等价于 STM32 InjectedStart_IT */
  adc_interrupt_enable(ADC2, ADC_PCCE_INT, TRUE);
  return ERR_NONE;
}

error_t current_hw_deinit(void)
{
  adc_interrupt_enable(ADC2, ADC_PCCE_INT, FALSE);
  return ERR_NONE;
}

error_t pwm_hw_init(void)
{
  /* 关闭三相调制输出，compare 清零（保持定时器运行以便 TMR1CH4 触发 ADC） */
  motor_actuation_t actuation = {
    .duty_a = 0,
    .duty_b = 0,
    .duty_c = 0,
  };
  set_pwm(&actuation);

  pwm_compare_top = pwm_get_compare_top();

  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, FALSE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, FALSE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, FALSE);

  /* 与 STM32 侧 HAL_TIM_Base_Start_IT / OC4_IT 对应：若后续挂接 TMR1 NVIC，可在此处使能 TMR_OVF_INT / TMR_C4_INT */
  return ERR_NONE;
}

error_t pwm_hw_deinit(void)
{
  /* 工程未注册 TMR1 中断服务程序时不打开定时器中断；停机语义留给 pwm_hw_stop */
  motor_actuation_t actuation = {
    .duty_a = 0.0f,
    .duty_b = 0.0f,
    .duty_c = 0.0f,
  };
  set_pwm(&actuation);

  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, FALSE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, FALSE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, FALSE);
  return ERR_NONE;
}

error_t pwm_hw_start(void)
{
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, TRUE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, TRUE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, TRUE);
  return ERR_NONE;
}

error_t pwm_hw_stop(void)
{
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, FALSE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, FALSE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, FALSE);
  return ERR_NONE;
}

/**
 * @brief 打开低侧刹车
 * 
 * 低侧刹车：将三相 PWM 占空比设置为比较值，占空比为 100%。
 * 此时，上桥臂导通，下桥臂关断，电机处于自由制动状态。
 * 
 * @return error_t 
 */
error_t pwm_hw_lowside_brake_on(void)
{
  if (pwm_compare_top == 0U)
  {
    pwm_compare_top = pwm_get_compare_top();
  }

  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, (uint32_t)pwm_compare_top);
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, (uint32_t)pwm_compare_top);
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, (uint32_t)pwm_compare_top);
  return ERR_NONE;
}

/**
 * @brief 关闭低侧刹车
 * 
 * 低侧刹车：将三相 PWM 占空比设置为 0，占空比为 0%。
 * 此时，上桥臂关断，下桥臂导通，电机处于低侧刹车状态。
 * 
 * @return error_t 
 */
error_t pwm_hw_lowside_brake_off(void)
{
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, 0U);
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, 0U);
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, 0U);
  return ERR_NONE;
}

/*
 * @brief 设置 PWM 占空比
 * @param actuation 电机控制占空比结构体
 * @param duty_a A 相 PWM 占空比 [0, 1]
 * @param duty_b B 相 PWM 占空比 [0, 1]
 * @param duty_c C 相 PWM 占空比 [0, 1]
 * @return 错误码
 */
error_t set_pwm(motor_actuation_t *actuation)
{
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL,
                        (uint32_t)(actuation->duty_a * pwm_compare_top));
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL,
                        (uint32_t)(actuation->duty_b * pwm_compare_top));
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL,
                        (uint32_t)(actuation->duty_c * pwm_compare_top));
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

  s->board_temp.adc_raw = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_1);
  s->phase_current.adc_raw[0] = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_2);
  s->phase_current.adc_raw[1] = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_3);
  s->phase_current.adc_raw[2] = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_4);
}

void board_current_loop_irq_handler(void *adc_handle)
{
  adc_type *adc_x = (adc_type *)adc_handle;

  /* ADC2 定义为 ((adc_type *)ADC2_BASE)，应与 ISR 传入指针同一实例 */
  if (adc_x == ADC2)
  {
    board_invoke_current_loop();
  }
}

static void board_uart_dma_init_once(void)
{
  if (s_board_uart_dma_inited != 0u)
  {
    return;
  }

  dma_interrupt_enable(DMA1_CHANNEL3, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL3, DMA_DTERR_INT, TRUE);
  s_board_uart_dma_inited = 1u;
}

void board_uart_stream_mode_set(uint8_t enabled)
{
  board_uart_dma_init_once();
  s_board_uart_stream_mode = (enabled != 0u) ? 1u : 0u;
}

uint8_t board_uart_tx_try(const uint8_t *data, uint16_t len)
{
  uint32_t primask;

  if ((data == NULL) || (len == 0u))
  {
    return 0u;
  }

  if (len > BOARD_UART_RAW_TX_BUF_SIZE)
  {
    len = BOARD_UART_RAW_TX_BUF_SIZE;
  }

  board_uart_dma_init_once();
  primask = __get_PRIMASK();
  __disable_irq();
  if (s_board_uart_busy != 0u)
  {
    if (primask == 0u)
    {
      __enable_irq();
    }
    return 0u;
  }

  memcpy(s_board_uart_raw_buf, data, len);
  s_board_uart_busy = 1u;
  s_board_uart_tx_len = len;
  dma_channel_enable(DMA1_CHANNEL3, FALSE);
  dma_flag_clear(DMA1_GL3_FLAG);
  wk_dma_channel_config(DMA1_CHANNEL3, (uint32_t)&USART1->dt, (uint32_t)s_board_uart_raw_buf, len);
  dma_channel_enable(DMA1_CHANNEL3, TRUE);

  if (primask == 0u)
  {
    __enable_irq();
  }
  return 1u;
}

uint8_t board_uart_log_try(const uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0u))
  {
    return 0u;
  }

  if (s_board_uart_stream_mode != 0u)
  {
    s_board_uart_dropped_messages += 1u;
    s_board_uart_dropped_bytes += len;
    return 0u;
  }

  if (board_uart_tx_try(data, len) == 0u)
  {
    s_board_uart_dropped_messages += 1u;
    s_board_uart_dropped_bytes += len;
    return 0u;
  }

  return 1u;
}

uint8_t board_uart_tx_busy(void)
{
  if (s_board_uart_busy != 0u)
  {
    return 1u;
  }
  return 0u;
}

void board_uart_dma_irq_handler(void)
{
  uint32_t primask;

  board_uart_dma_init_once();
  if (dma_flag_get(DMA1_DTERR3_FLAG) != RESET)
  {
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
    dma_flag_clear(DMA1_GL3_FLAG);
    s_board_uart_busy = 0u;
    s_board_uart_tx_len = 0u;
    return;
  }

  if (dma_flag_get(DMA1_FDT3_FLAG) == RESET)
  {
    return;
  }

  dma_channel_enable(DMA1_CHANNEL3, FALSE);
  dma_flag_clear(DMA1_GL3_FLAG);
  primask = __get_PRIMASK();
  __disable_irq();
  s_board_uart_busy = 0u;
  s_board_uart_tx_len = 0u;
  if (primask == 0u)
  {
    __enable_irq();
  }
}

void board_uart_get_diag(board_uart_diag_t *diag)
{
  uint32_t primask;

  if (diag == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  diag->ring_size = 0u;
  diag->used = 0u;
  diag->free = 0u;
  diag->tx_inflight = s_board_uart_tx_len;
  diag->dma_busy = board_uart_tx_busy();
  diag->dropped_messages = s_board_uart_dropped_messages;
  diag->dropped_bytes = s_board_uart_dropped_bytes;
  if (primask == 0u)
  {
    __enable_irq();
  }
}
