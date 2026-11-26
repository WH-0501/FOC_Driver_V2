#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "datatypes.h"
#include "at32_foc_timing.h"
static struct { struct { uint32_t ovfen; } ctrl1_bit; uint32_t c[3]; } timer;
static unsigned mask, inject, enabled[3];
static uint32_t active[3];
#define PWM_TIM_HANDLE (&timer)
#define PWM_TIME_U_CHANNEL 2u
#define PWM_TIME_V_CHANNEL 1u
#define PWM_TIME_W_CHANNEL 0u
#define TRUE 1
#define FALSE 0
#define __get_PRIMASK() mask
#define __disable_irq() (mask=1)
#define __enable_irq() (mask=0)
static void update(void) { if(!timer.ctrl1_bit.ovfen) memcpy(active,timer.c,sizeof(active)); }
#define tmr_channel_enable(p,c,on) (enabled[c]=(on))
static void tmr_channel_value_set(void *p,unsigned channel,uint32_t value) {
  assert(p==&timer && channel<3);timer.c[channel]=value;if(inject)update();
}
/* Exact production predicate, guarded writer, stop and set_pwm; MMIO only mocked. */
#include "pwm_impl.inc"
int main(void) {
  motor_actuation_t pwm={0};float limit=AT32_FOC_CURRENT_MAX_DUTY;
  uint32_t seed=0x12345678u,ceiling,bits;unsigned i,phase;
  const uint32_t edge[]={0u,0x80000000u,1u,0x80000001u,0x007fffffu,0x807fffffu,
    0x00800000u,0x80800000u,0x7f7fffffu,0xff7fffffu,0x7f800000u,0xff800000u,
    0x7f800001u,0xff800001u,0x7fc00000u,0xffc00000u};
  memcpy(&ceiling,&limit,4);
  assert(!pwm_at32_duties_in_range(NULL,limit));
  for(i=0;i<2000145u;++i) {
    if(i<16u)bits=edge[i];
    else if(i<145u)bits=ceiling+i-16u-64u;
    else {seed=1664525u*seed+1013904223u;bits=seed;}
    for(phase=0;phase<3;++phase) {
      bool valid;
      pwm.duty_a=.2f;pwm.duty_b=.3f;pwm.duty_c=.4f;
      if(phase==0)memcpy(&pwm.duty_a,&bits,4);
      else if(phase==1)memcpy(&pwm.duty_b,&bits,4);
      else memcpy(&pwm.duty_c,&bits,4);
      valid=pwm_duties_in_range(&pwm,limit);
      assert(pwm_at32_duties_in_range(&pwm,limit)==valid);
      enabled[0]=enabled[1]=enabled[2]=1;
      assert(set_pwm(&pwm,limit)==(valid?ERR_NONE:ERR_PARAM));
      if(valid) {
        assert(timer.c[2]==(uint32_t)(pwm.duty_a*AT32_FOC_TIMER_ARR));
        assert(timer.c[1]==(uint32_t)(pwm.duty_b*AT32_FOC_TIMER_ARR));
        assert(timer.c[0]==(uint32_t)(pwm.duty_c*AT32_FOC_TIMER_ARR));
      } else {
        assert(!enabled[0]&&!enabled[1]&&!enabled[2]);
        assert(!timer.c[0]&&!timer.c[1]&&!timer.c[2]);
      }
    }
  }
  pwm.duty_a=.2f;pwm.duty_b=.3f;pwm.duty_c=.4f;
  assert(set_pwm(NULL,limit)==ERR_PARAM);
  assert(set_pwm(&pwm,1.0f)==ERR_PARAM);
  assert(set_pwm(&pwm,NAN)==ERR_PARAM);
  assert(set_pwm(&pwm,INFINITY)==ERR_PARAM);
  assert(set_pwm(&pwm,limit)==ERR_NONE);update();
  inject=1;pwm.duty_b=.1f;
  {uint32_t previous[3];memcpy(previous,active,sizeof(active));
    assert(set_pwm(&pwm,limit)==ERR_NONE);
    assert(!memcmp(previous,active,sizeof(active)));
    assert(mask==0 && timer.ctrl1_bit.ovfen==0);update();assert(active[1]==timer.c[1]);}
  mask=1;timer.ctrl1_bit.ovfen=1;
  assert(set_pwm(&pwm,limit)==ERR_NONE);
  assert(mask==1 && timer.ctrl1_bit.ovfen==1);
  printf("PASS PWM direct=%d: 6000435 predicate/output comparisons; invalid stop, atomic update and IRQ-state preservation\n",AT32_PWM_INLINE_STAGE_CCR);
}
