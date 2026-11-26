#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc.h"
#include "motor.h"
#include "foc_fault.h"
#include "at32_foc_timing.h"
static unsigned writes, stops;
static motor_actuation_t last;
void motor_fault_raise(motor_handle_t *m, fault_t bits) { m->fault.latched |= bits; m->fault.status |= bits; }
void motor_disarm(motor_handle_t *m) { m->out.pwm_enable = 0; ++stops; }
error_t motor_apply_pwm(motor_handle_t *m, const motor_actuation_t *p, float limit)
{
  (void)m; assert(limit == AT32_FOC_CURRENT_MAX_DUTY); last=*p; ++writes; return ERR_NONE;
}
#include "../project/APP/FOC/foc.c"
int main(void)
{
  motor_handle_t m; memset(&m,0,sizeof(m)); m.state.vbus_filtered=24; m.out.pwm_enable=1;
  foc_voltage(&m,100,0,1);
  assert(writes==1 && stops==0);
  assert(last.duty_a>=0 && last.duty_a<=AT32_FOC_CURRENT_MAX_DUTY);
  assert(last.duty_b>=0 && last.duty_b<=AT32_FOC_CURRENT_MAX_DUTY);
  assert(last.duty_c>=0 && last.duty_c<=AT32_FOC_CURRENT_MAX_DUTY);
  foc_voltage(&m,NAN,0,0); assert(writes==1 && stops==1 && (m.fault.latched & FAULT_PARAM));
  foc_voltage(&m,1,0,INFINITY); assert(writes==1 && stops==2);
  foc_voltage(NULL,0,0,0); assert(writes==1 && stops==2);
  puts("PASS: actual foc_voltage saturation and invalid-input fault/disarm"); return 0;
}
