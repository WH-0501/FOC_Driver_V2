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

extern void foc_control_loop(void);

#define PWM_TIM_HANDLE TMR1
#define PWM_TIME_U_CHANNEL TMR_SELECT_CHANNEL_1
#define PWM_TIME_V_CHANNEL TMR_SELECT_CHANNEL_2
#define PWM_TIME_W_CHANNEL TMR_SELECT_CHANNEL_3

uint16_t pwm_compare_top = 0;

static uint16_t pwm_compare_top(void)
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

  pwm_compare_top = pwm_compare_top();

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
    .duty_a = 0,
    .duty_b = 0,
    .duty_c = 0,
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

  board_apply_phase_current(m);
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
