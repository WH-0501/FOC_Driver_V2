/* Execute the real AT32 board implementation against a RAM timer, with the
 * vendor's actual register layout and two timer write functions stubbed. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "board.h"
#include "at32m412_416_wk_config.h"
#include "at32m412_416_adc.h"
#include "at32m412_416_tmr.h"
#include "wk_dma.h"
#include "wk_usart.h"

static tmr_type timer_ram;
static unsigned int disabled_channels;
static uint32_t irq_mask;
static uint32_t active_compare[3];
static unsigned int inject_update;
static unsigned int channel_enabled[3];
uint32_t SystemCoreClock = 180000000u;
#undef TMR1
#define TMR1 (&timer_ram)
/* The unrelated UART functions in this translation unit use ARM intrinsics.
 * Replace only those intrinsics for the x86 host test, not the PWM functions. */
#define __get_PRIMASK() irq_mask
#define __disable_irq() (irq_mask = 1u)
#define __enable_irq() (irq_mask = 0u)
#include "../project/Board/board_at32.c"

static void natural_update(void)
{
  if (!timer_ram.ctrl1_bit.ovfen)
  {
    active_compare[0] = timer_ram.c1dt;
    active_compare[1] = timer_ram.c2dt;
    active_compare[2] = timer_ram.c3dt;
  }
}
void tmr_output_channel_buffer_enable(tmr_type *timer, tmr_channel_select_type channel, confirm_state state)
{
  assert(timer == &timer_ram);
  switch (channel)
  {
    case TMR_SELECT_CHANNEL_1: timer->cm1_output_bit.c1oben = state; break;
    case TMR_SELECT_CHANNEL_2: timer->cm1_output_bit.c2oben = state; break;
    case TMR_SELECT_CHANNEL_3: timer->cm2_output_bit.c3oben = state; break;
    default: assert(0);
  }
}
void tmr_channel_value_set(tmr_type *timer, tmr_channel_select_type channel, uint32_t value)
{
  assert(timer == &timer_ram);
  switch (channel)
  {
    case TMR_SELECT_CHANNEL_1: timer->c1dt = value; if (!timer->cm1_output_bit.c1oben) active_compare[0] = value; break;
    case TMR_SELECT_CHANNEL_2: timer->c2dt = value; if (!timer->cm1_output_bit.c2oben) active_compare[1] = value; break;
    case TMR_SELECT_CHANNEL_3: timer->c3dt = value; if (!timer->cm2_output_bit.c3oben) active_compare[2] = value; break;
    default: assert(0);
  }
  if (inject_update) natural_update();
}
void tmr_channel_enable(tmr_type *timer, tmr_channel_select_type channel, confirm_state state)
{
  assert(timer == &timer_ram);
  channel_enabled[channel / 2] = state;
  if (!state) ++disabled_channels;
}

static void expect_reject(motor_actuation_t *pwm)
{
  unsigned int before = disabled_channels;
  assert(set_pwm(pwm) == ERR_PARAM);
  assert(disabled_channels == before + 3);
  assert(timer_ram.c1dt == 0 && timer_ram.c2dt == 0 && timer_ram.c3dt == 0);
}

int main(void)
{
  motor_actuation_t pwm = {0};
  float max_duty;
  timer_ram.pr = AT32_FOC_TIMER_ARR;
  timer_ram.div = AT32_FOC_TIMER_DIV_REG;
  timer_ram.c4dt = AT32_FOC_ADC_TRIGGER_CCR;
  tmr_output_channel_buffer_enable(&timer_ram, TMR_SELECT_CHANNEL_1, TRUE);
  tmr_output_channel_buffer_enable(&timer_ram, TMR_SELECT_CHANNEL_2, TRUE);
  tmr_output_channel_buffer_enable(&timer_ram, TMR_SELECT_CHANNEL_3, TRUE);
  max_duty = pwm_hw_max_duty();
  assert(max_duty == AT32_FOC_CURRENT_MAX_DUTY);
  assert(pwm_compare_top == timer_ram.pr); /* lazy initialization */
  pwm.duty_a = max_duty;
  pwm.duty_b = max_duty * .5f;
  pwm.duty_c = 0;
  assert(set_pwm(&pwm) == ERR_NONE);
  assert(timer_ram.c3dt <= AT32_FOC_CURRENT_MAX_CCR);
  assert(timer_ram.c3dt == (uint32_t)(max_duty * AT32_FOC_TIMER_ARR));
  assert(timer_ram.c2dt == (uint32_t)(max_duty * .5f * AT32_FOC_TIMER_ARR));
  assert(timer_ram.c1dt == 0);
  natural_update();
  {
    uint32_t old[3] = {active_compare[0], active_compare[1], active_compare[2]};
    inject_update = 1;
    pwm.duty_b *= .5f;
    assert(set_pwm(&pwm) == ERR_NONE);
    assert(active_compare[0] == old[0] && active_compare[1] == old[1] && active_compare[2] == old[2]);
    assert(irq_mask == 0 && timer_ram.ctrl1_bit.ovfen == 0);
    natural_update();
    assert(active_compare[1] == timer_ram.c2dt && active_compare[1] != old[1]);
    irq_mask = 1;
    timer_ram.ctrl1_bit.ovfen = 1;
    assert(set_pwm(&pwm) == ERR_NONE);
    assert(irq_mask == 1 && timer_ram.ctrl1_bit.ovfen == 1);
    irq_mask = 0;
    timer_ram.ctrl1_bit.ovfen = 0;
    inject_update = 0;
  }
  /* A stale cache must fail closed, rather than mis-scale an otherwise valid
   * duty. Board initialization explicitly rebinds it to the actual ARR. */
  pwm_compare_top--;
  assert(pwm_hw_max_duty() == 0);
  expect_reject(&pwm);
  board_init();
  assert(pwm_compare_top == timer_ram.pr);
  assert(set_pwm(&pwm) == ERR_NONE);
  pwm.duty_a = nextafterf(max_duty, INFINITY);
  expect_reject(&pwm);
  pwm.duty_a = NAN;
  expect_reject(&pwm);
  pwm.duty_a = INFINITY;
  expect_reject(&pwm);
  pwm.duty_a = -0.01f;
  expect_reject(&pwm);
  expect_reject(NULL);
  pwm.duty_a = 0;
  timer_ram.c4dt++;
  assert(pwm_hw_max_duty() == 0);
  expect_reject(&pwm);
  timer_ram.c4dt--;
  timer_ram.div++;
  assert(pwm_hw_max_duty() == 0);
  timer_ram.div--;
  timer_ram.pr++;
  assert(pwm_hw_max_duty() == 0);
  expect_reject(&pwm);
  assert(pwm_compare_top == AT32_FOC_TIMER_ARR);
  timer_ram.pr--;
  SystemCoreClock /= 2;
  assert(pwm_hw_max_duty() == 0);
  SystemCoreClock *= 2;
  timer_ram.c1dt = timer_ram.c2dt = timer_ram.c3dt = 123;
  assert(pwm_hw_lowside_brake_on() == ERR_NONE);
  assert(timer_ram.c1dt == 0 && timer_ram.c2dt == 0 && timer_ram.c3dt == 0);
  timer_ram.c1dt = timer_ram.c2dt = timer_ram.c3dt = 500;
  natural_update();
  assert(pwm_hw_start() == ERR_NONE);
  assert(active_compare[0] == 0 && active_compare[1] == 0 && active_compare[2] == 0);
  assert(timer_ram.c1dt == 0 && timer_ram.c2dt == 0 && timer_ram.c3dt == 0);
  assert(timer_ram.cm1_output_bit.c1oben && timer_ram.cm1_output_bit.c2oben && timer_ram.cm2_output_bit.c3oben);
  assert(channel_enabled[0] && channel_enabled[1] && channel_enabled[2]);
  assert(irq_mask == 0);
  puts("PASS: real board PWM guard, phase mapping, mismatch rejection and low-side calibration");
  return 0;
}
