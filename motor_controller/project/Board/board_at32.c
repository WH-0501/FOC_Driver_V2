/*
 * AT32M416：工程中关闭 board_stm32.c。
 * ADC/TMR 外设与时基由 WorkBench（wk_adc*.c / wk_tmr.c）初始化；本文件负责电流环抢占采样中断开关、
 * PWM 三相通道启停与占空比写入，语义对齐 board_stm32.c。
 *
 * PWM 计数周期与 wk_tmr1_init() 中 tmr_base_init(TMR1, pr, ...) 的 pr 一致，占空比按 compare/pr 缩放。
 */
#include "board.h"
#include "foc.h"
#include "at32m412_416_adc.h"
#include "at32m412_416_tmr.h"
#include "wk_dma.h"
#include "wk_usart.h"
#include <string.h>

extern void foc_control_loop(void);

#define PWM_TIM_HANDLE TMR1
#define PWM_TIME_U_CHANNEL TMR_SELECT_CHANNEL_1
#define PWM_TIME_V_CHANNEL TMR_SELECT_CHANNEL_2
#define PWM_TIME_W_CHANNEL TMR_SELECT_CHANNEL_3

uint16_t pwm_compare_top = 0;

#ifndef BOARD_UART_DMA_RING_SIZE
#define BOARD_UART_DMA_RING_SIZE 1024u
#endif

#ifndef BOARD_UART_RAW_TX_BUF_SIZE
#define BOARD_UART_RAW_TX_BUF_SIZE 256u
#endif

static uint8_t s_board_uart_dma_inited;
static volatile uint8_t s_board_uart_stream_mode;
static uint8_t s_board_uart_raw_buf[BOARD_UART_RAW_TX_BUF_SIZE];
static uint8_t s_board_uart_log_ring[BOARD_UART_DMA_RING_SIZE];
static volatile uint16_t s_board_uart_log_head;
static volatile uint16_t s_board_uart_log_tail;
static volatile uint16_t s_board_uart_log_tx_len;
static volatile uint8_t s_board_uart_log_busy;
static volatile uint8_t s_board_uart_raw_busy;
static volatile uint32_t s_board_uart_dropped_messages;
static volatile uint32_t s_board_uart_dropped_bytes;

static uint16_t pwm_get_compare_top(void)
{
  return (uint16_t)PWM_TIM_HANDLE->pr;
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
    foc_control_loop();
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

static uint16_t board_uart_ring_available(void)
{
  uint16_t head;
  uint16_t tail;

  head = s_board_uart_log_head;
  tail = s_board_uart_log_tail;
  if (head >= tail)
  {
    return (uint16_t)(BOARD_UART_DMA_RING_SIZE - (head - tail) - 1u);
  }
  return (uint16_t)(tail - head - 1u);
}

static uint16_t board_uart_ring_used(void)
{
  uint16_t head;
  uint16_t tail;

  head = s_board_uart_log_head;
  tail = s_board_uart_log_tail;
  if (head >= tail)
  {
    return (uint16_t)(head - tail);
  }
  return (uint16_t)(BOARD_UART_DMA_RING_SIZE - (tail - head));
}

static void board_uart_start_log_tx_locked(void)
{
  uint16_t head;
  uint16_t tail;
  uint16_t tx_len;

  if ((s_board_uart_log_busy != 0u) || (s_board_uart_raw_busy != 0u))
  {
    return;
  }
  if (s_board_uart_stream_mode != 0u)
  {
    return;
  }

  head = s_board_uart_log_head;
  tail = s_board_uart_log_tail;
  if (head == tail)
  {
    return;
  }

  if (head > tail)
  {
    tx_len = (uint16_t)(head - tail);
  }
  else
  {
    tx_len = (uint16_t)(BOARD_UART_DMA_RING_SIZE - tail);
  }

  if (tx_len == 0u)
  {
    return;
  }

  s_board_uart_log_tx_len = tx_len;
  s_board_uart_log_busy = 1u;
  dma_channel_enable(DMA1_CHANNEL3, FALSE);
  dma_flag_clear(DMA1_GL3_FLAG);
  wk_dma_channel_config(DMA1_CHANNEL3, (uint32_t)&USART1->dt, (uint32_t)&s_board_uart_log_ring[tail], tx_len);
  dma_channel_enable(DMA1_CHANNEL3, TRUE);
}

void board_uart_stream_mode_set(uint8_t enabled)
{
  uint32_t primask;

  board_uart_dma_init_once();
  primask = __get_PRIMASK();
  __disable_irq();
  s_board_uart_stream_mode = (enabled != 0u) ? 1u : 0u;
  if (enabled != 0u)
  {
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
    dma_flag_clear(DMA1_GL3_FLAG);
    s_board_uart_log_head = 0u;
    s_board_uart_log_tail = 0u;
    s_board_uart_log_tx_len = 0u;
    s_board_uart_log_busy = 0u;
    s_board_uart_raw_busy = 0u;
  }
  else
  {
    board_uart_start_log_tx_locked();
  }
  if (primask == 0u)
  {
    __enable_irq();
  }
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
  if ((s_board_uart_raw_busy != 0u) || (s_board_uart_log_busy != 0u))
  {
    if (primask == 0u)
    {
      __enable_irq();
    }
    return 0u;
  }

  memcpy(s_board_uart_raw_buf, data, len);
  s_board_uart_raw_busy = 1u;
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
  uint32_t primask;
  uint16_t free_space;
  uint16_t i;

  if ((data == NULL) || (len == 0u))
  {
    return 0u;
  }

  board_uart_dma_init_once();
  primask = __get_PRIMASK();
  __disable_irq();
  if ((s_board_uart_stream_mode != 0u) || (s_board_uart_raw_busy != 0u))
  {
    s_board_uart_dropped_messages += 1u;
    s_board_uart_dropped_bytes += len;
    if (primask == 0u)
    {
      __enable_irq();
    }
    return 0u;
  }

  free_space = board_uart_ring_available();
  if (free_space < len)
  {
    s_board_uart_dropped_messages += 1u;
    s_board_uart_dropped_bytes += (uint32_t)(len - free_space);
    len = free_space;
  }

  for (i = 0u; i < len; ++i)
  {
    s_board_uart_log_ring[s_board_uart_log_head] = data[i];
    s_board_uart_log_head = (uint16_t)((s_board_uart_log_head + 1u) % BOARD_UART_DMA_RING_SIZE);
  }

  board_uart_start_log_tx_locked();
  if (primask == 0u)
  {
    __enable_irq();
  }
  return (len > 0u) ? 1u : 0u;
}

uint8_t board_uart_tx_busy(void)
{
  if ((s_board_uart_raw_busy != 0u) || (s_board_uart_log_busy != 0u))
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
    s_board_uart_raw_busy = 0u;
    s_board_uart_log_busy = 0u;
    s_board_uart_log_tx_len = 0u;
    return;
  }

  if (dma_flag_get(DMA1_FDT3_FLAG) == RESET)
  {
    return;
  }

  dma_channel_enable(DMA1_CHANNEL3, FALSE);
  dma_flag_clear(DMA1_GL3_FLAG);

  if (s_board_uart_raw_busy != 0u)
  {
    s_board_uart_raw_busy = 0u;
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  s_board_uart_log_tail = (uint16_t)((s_board_uart_log_tail + s_board_uart_log_tx_len) % BOARD_UART_DMA_RING_SIZE);
  s_board_uart_log_tx_len = 0u;
  s_board_uart_log_busy = 0u;
  board_uart_start_log_tx_locked();
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
  diag->ring_size = BOARD_UART_DMA_RING_SIZE;
  diag->used = board_uart_ring_used();
  diag->free = board_uart_ring_available();
  diag->tx_inflight = s_board_uart_log_tx_len;
  diag->dma_busy = board_uart_tx_busy();
  diag->dropped_messages = s_board_uart_dropped_messages;
  diag->dropped_bytes = s_board_uart_dropped_bytes;
  if (primask == 0u)
  {
    __enable_irq();
  }
}
