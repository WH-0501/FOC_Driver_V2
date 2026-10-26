/*
 * AT32M416 板级实现。工程中编入本文件并定义 PLATFORM_AT32，勿同时编入其它 board_*.c。
 *
 * WorkBench 外设（wk_adc2 / wk_tmr1 / wk_tmr6 等）仍由 main.c 初始化；
 * 本文件负责电流环 IRQ、PWM 运行时控制、UART 与预驱 GPIO 绑定。
 */
#include "board.h"
#include "at32_foc_timing.h"
#include "at32m412_416_wk_config.h"
#include "at32m412_416_adc.h"
#include "at32m412_416_tmr.h"
#include "wk_dma.h"
#include "wk_usart.h"
#include <string.h>

#define PWM_TIM_HANDLE TMR1
#define PWM_TIME_U_CHANNEL TMR_SELECT_CHANNEL_3
#define PWM_TIME_V_CHANNEL TMR_SELECT_CHANNEL_2
#define PWM_TIME_W_CHANNEL TMR_SELECT_CHANNEL_1
#define BOARD_TIME_US_HANDLE TMR6

uint16_t pwm_compare_top = 0;

#ifndef BOARD_UART_RAW_TX_BUF_SIZE
#define BOARD_UART_RAW_TX_BUF_SIZE 256u
#endif

/**
 * @brief 母线电压采样电路
 * VIN+ ──[R13 100K]──┬──[R17 100R]──→ Vbus (→ ADC)
 *                    │
 *                   [R14 10K]
 *                    │
 *                   PGND
 * 电阻分压 + 限流电阻(R17), 将母线电压衰减到 MCU ADC 可测范围(0~3.3V)
 * Vbus = (VIN * R14) / (R13 + R14) ≈ 0.0909 x VIN
 * Vadc = (ADC_value / 4095) * 3.3V = ADC_value * 3.3 / 4095
 * (注意: 工程上除以 4096 便于移位运算，除以 4095 是理论满量程，差别极微)
 */
#define _ADC_CONV       (0.000805608f) // 3.3/4095
#define VOLTAGE_DIVIDER_RATIO (11) // 电压衰减比(分压比)
#define _ADC_TEMP_SLOPE (-0.0042f)


static uint8_t s_board_uart_dma_inited;
static volatile uint8_t s_board_uart_stream_mode;
static uint8_t s_board_uart_raw_buf[BOARD_UART_RAW_TX_BUF_SIZE];
static volatile uint8_t s_board_uart_busy;
static volatile uint16_t s_board_uart_tx_len;
static volatile uint32_t s_board_uart_dropped_messages;
static volatile uint32_t s_board_uart_dropped_bytes;

volatile uint16_t s_adc1_raw[2] = {0};

static uint16_t pwm_get_compare_top(void)
{
  return (uint16_t)PWM_TIM_HANDLE->pr;
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
  return (uint32_t)BOARD_TIME_US_HANDLE->cval;
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
  adc_interrupt_enable(ADC2, ADC_PCCE_INT, TRUE);
  return ERR_NONE;
}

error_t current_hw_deinit(void)
{
  adc_interrupt_enable(ADC2, ADC_PCCE_INT, FALSE);
  return ERR_NONE;
}

/**
 * @brief 获取 PWM 最大占空比
 * 
 * 为了给 ADC 留出可靠的低侧电流采样窗口，当前允许输出的最大高侧占空比。
 * 返回的约 84.84% 来自 PWM 时序、ADC 转换耗时和建立裕量的共同约束。
 * 
 * 注：当前 ADC 时钟为 30MHz，定时器计数频率为 90MHz。
 * 
 * 原理分析：
 *   当前采用中心对齐 PWM，计数器往返运行： CNT=0 → ARR → 0 → ...
 *   在当前有效高 PWM A 配置下：
 *         CNT < CCR：PWM 高，高侧导通
 *         CNT > CCR：PWM 低，低侧导通
 *   MP6540 的相电流反馈来自低侧 MOSFET。因此，需要在低侧导通且信号稳定后采样。
 *   CCR 越接近 ARR，高侧占空比越大，峰值附近的低侧导通窗口越短。 
 *   如果允许占空比接近 100%，ADC 就可能采到尚未稳定或无效的电流信号。
 * 
 *   三相共同可采样的窗口，由三相中最大的 CCR 决定：
 *         共同低侧窗口长度 ≈ 2 × (ARR − max(CCRa, CCRb, CCRc))
 *   
 *   ADC 注入序列包括 Ia、Ib、Ic、温度，共四个通道。当前保守地要求整个序列都位于窗口内。
 *   每个通道需要： 13.5 个采样周期 + 12.5 个转换周期 = 26 个 ADC 周期
 *   再加上代码预留的 3 个触发与同步周期：完整序列预算 = 4 × 26 + 3 = 107 个 ADC 周期
 *                                     时间序列 = 107 / 30MHz = 3.56us 
 *                                     换算为定时器 ticks：107 x 90MHz / 30MHz = 321 ticks
 *   此外，当前在序列前后分别预留 2 μs：settling_ticks = 2 μs × 90 MHz = 180 tick
 * 
 *   1.为什么 ADC 触发点是 2088?
 *   为了将整个 ADC 序列大致居中放在计数器峰值附近，触发点提前半个序列长度：
 *         trigger_CCR = ARR - ceil(sequence_ticks / 2) = 2249 - ceil(321 / 2) = 2088
 * 
 *   2.为什么 CCR 最大是 1908?
 *   ADC 开始前，三相低侧必须已经共同导通至少 180 tick：trigger_CCR − maxCCR ≥ settling_ticks
 *   因此，maxCCR = trigger_CCR - settling_ticks = 2088 - 180 = 1908
 *   还要检查序列结束后，距离下一个开关边沿是否足够远。以“从 CNT=0 开始经过的 tick”为统一时间轴：
 *          低侧窗口开始：1908
 *          ADC 触发：    2088
 *          计数器峰值：  2249
 *          序列预算结束：2088 + 321 = 2409
 *          低侧窗口结束：2 × 2249 − 1908 = 2590
 *   所以两侧实际裕量分别是：
 *         触发前：2088 − 1908 = 180 tick = 2.000 μs
 *         结束后：2590 − 2409 = 181 tick ≈ 2.011 μs
 *   因此，maxCCR = 1908 是安全的。
 * 
 * @return 最大占空比
 */
float pwm_hw_max_duty(void)
{
  uint16_t actual_top = pwm_get_compare_top();
  /* Refuse a regenerated/mismatched timer setup instead of using stale limits.
   * Clock, ADC divider and sampling-time changes must update the shared contract. */
  if (SystemCoreClock != AT32_FOC_TIMER_CLOCK_HZ ||
      actual_top != AT32_FOC_TIMER_ARR ||
      PWM_TIM_HANDLE->div != AT32_FOC_TIMER_DIV_REG ||
      PWM_TIM_HANDLE->c4dt != AT32_FOC_ADC_TRIGGER_CCR)
  {
    return 0.0f;
  }
  /* Keep the PWM conversion based on the actual timer period. A zero cache
   * can be initialized lazily; a stale nonzero cache must never be used. */
  if (pwm_compare_top == 0U)
  {
    pwm_compare_top = actual_top;
  }
  if (pwm_compare_top != actual_top)
  {
    return 0.0f;
  }
  return (float)AT32_FOC_CURRENT_MAX_CCR / (float)pwm_compare_top;
}

error_t pwm_hw_init(void)
{
  motor_actuation_t actuation = {
    .duty_a = 0.0f,
    .duty_b = 0.0f,
    .duty_c = 0.0f,
  };

  if (pwm_compare_top == 0U)
  {
    pwm_compare_top = pwm_get_compare_top();
  }

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

error_t pwm_hw_lowside_brake_off(void)
{
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, 0U);
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, 0U);
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, 0U);
  return ERR_NONE;
}

error_t pwm_hw_stop(void)
{
  (void)pwm_hw_lowside_brake_off();
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, FALSE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, FALSE);
  tmr_channel_enable(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, FALSE);
  return ERR_NONE;
}

error_t pwm_hw_lowside_brake_on(void)
{
  /* MP6540: EN=1, PWM=0 selects the low-side FET. With active-high
   * PWM A, CCR=0 keeps PWM low, including the current-offset calibration. */
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL, 0U);
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL, 0U);
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL, 0U);
  return ERR_NONE;
}

error_t set_pwm(motor_actuation_t *actuation)
{
  float max_duty = pwm_hw_max_duty();

  /* Ordered comparisons reject NaN; bounded range rejects infinity. Never
   * silently clip individual phases, because that changes the voltage vector. */
  if (actuation == NULL || !(max_duty > 0.0f) ||
      !(actuation->duty_a >= 0.0f && actuation->duty_a <= max_duty) ||
      !(actuation->duty_b >= 0.0f && actuation->duty_b <= max_duty) ||
      !(actuation->duty_c >= 0.0f && actuation->duty_c <= max_duty))
  {
    (void)pwm_hw_stop();
    return ERR_PARAM;
  }

  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_U_CHANNEL,
                        (uint32_t)(actuation->duty_a * (float)pwm_compare_top));
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_V_CHANNEL,
                        (uint32_t)(actuation->duty_b * (float)pwm_compare_top));
  tmr_channel_value_set(PWM_TIM_HANDLE, PWM_TIME_W_CHANNEL,
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
  /* Keep this mapping aligned with wk_adc2_init: currents first, temperature last. */
  s->phase_current.adc_raw[0] = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_1);
  s->phase_current.adc_raw[1] = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_2);
  s->phase_current.adc_raw[2] = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_3);
  s->motor_temp.adc_raw = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_4);
}

void board_get_vbus_and_temp(motor_handle_t *m)
{
  motor_state_t *s;

  if (m == NULL)
  {
    return;
  }

  // adc_ordinary_software_trigger_enable(ADC1, TRUE);
  // while(adc_flag_get(ADC1, ADC_OCCE_FLAG) == RESET);

  s = &m->state;
  s->vbus = (float)s_adc1_raw[0] * _ADC_CONV * VOLTAGE_DIVIDER_RATIO;
  s->board_temp.adc_raw = s_adc1_raw[1];
  s->board_temp.value = 
    ((float)(m->config.temp_adc - s->board_temp.adc_raw) * _ADC_CONV) / _ADC_TEMP_SLOPE + m->config.temp;

  // adc_flag_clear(ADC1, ADC_OCCE_FLAG);
  adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

void board_current_loop_irq_handler(void *adc_handle)
{
  adc_type *adc_x = (adc_type *)adc_handle;

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
  return (s_board_uart_busy != 0u) ? 1u : 0u;
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
