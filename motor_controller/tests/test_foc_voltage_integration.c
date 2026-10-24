#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc.h"
#include "motor.h"
#include "foc_fault.h"
#include "at32_foc_timing.h"

static float capability;
static unsigned int writes, stops;
static motor_actuation_t last;

float pwm_hw_max_duty(void) { return capability; }
void motor_fault_raise(motor_handle_t *m, fault_t bits)
{
  m->fault.latched |= bits;
  m->fault.status |= bits;
}
void motor_disarm(motor_handle_t *m) { m->out.pwm_enable = 0; ++stops; }
error_t motor_apply_pwm(motor_handle_t *m, const motor_actuation_t *actuation)
{
  (void)m;
  last = *actuation;
  ++writes;
  return ERR_NONE;
}

int main(void)
{
  motor_handle_t m;
  memset(&m, 0, sizeof(m));
  m.state.vbus_filtered = 24;
  m.out.pwm_enable = 1;
  capability = AT32_FOC_CURRENT_MAX_DUTY;
  foc_voltage(&m, 100, 0, 1);
  assert(writes == 1 && stops == 0);
  assert(last.duty_a >= 0 && last.duty_a <= capability);
  assert(last.duty_b >= 0 && last.duty_b <= capability);
  assert(last.duty_c >= 0 && last.duty_c <= capability);
  assert(m.inv_park_result.u_alpha != 0);

  capability = 0; /* mismatched hardware contract */
  foc_voltage(&m, 1, 0, 0);
  assert(writes == 1 && stops == 1);
  assert(m.fault.latched & FAULT_CURRENT_SENSOR);
  assert(m.out.pwm_enable == 0);
  capability = AT32_FOC_CURRENT_MAX_DUTY;
  foc_voltage(&m, NAN, 0, 0);
  assert(writes == 1 && stops == 2);
  assert(m.fault.latched & FAULT_PARAM);
  foc_voltage(NULL, 0, 0, 0);
  assert(writes == 1 && stops == 2);
  puts("PASS: real foc_voltage board capability, output bounds and fault/disarm integration");
  return 0;
}

