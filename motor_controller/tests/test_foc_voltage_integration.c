#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc.h"
#include "motor.h"
#include "foc_fault.h"
#include "at32_foc_timing.h"
/* Exercise the real private current path as well as the public voltage entry. */
#include "../project/APP/FOC/foc.c"

static uint32_t ticks;
uint32_t dwt_get_ticks(void) { return ticks; }
uint32_t dwt_get_ticks_us(void) { return ticks / 180u; }
uint32_t dwt_elapsed_us(uint32_t start, uint32_t end) { return (end-start) / 180u; }
static float capability;
static unsigned int writes, stops, limit_reads;
static motor_actuation_t last;

float pwm_hw_max_duty(void) { ++limit_reads; return capability; }
void motor_fault_raise(motor_handle_t *m, fault_t bits)
{
  m->fault.latched |= bits;
  m->fault.status |= bits;
}
void motor_disarm(motor_handle_t *m) { m->out.pwm_enable = 0; ++stops; }
error_t motor_apply_pwm(motor_handle_t *m, const motor_actuation_t *actuation, float max_duty)
{
  (void)m;
  assert(max_duty == capability);
  last = *actuation;
  ++writes;
  ticks += 720u;
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
  assert(writes == 1 && stops == 0 && limit_reads == 1);
  assert(last.duty_a >= 0 && last.duty_a <= capability);
  assert(last.duty_b >= 0 && last.duty_b <= capability);
  assert(last.duty_c >= 0 && last.duty_c <= capability);
  assert(m.inv_park_result.u_alpha != 0);

  /* An explicit zero angle in calibration works even before Park seeds its cache. */
  foc_voltage(&m, 2.0f, 0.0f, 0.0f);
  assert(writes == 2 && stops == 0);
  assert(m.inv_park_result.u_alpha == 0.0f && m.inv_park_result.u_beta == 2.0f);

  capability = 0; /* mismatched hardware contract */
  foc_voltage(&m, 1, 0, 0);
  assert(writes == 2 && stops == 1);
  assert(m.fault.latched & FAULT_CURRENT_SENSOR);
  assert(m.out.pwm_enable == 0);
  capability = AT32_FOC_CURRENT_MAX_DUTY;
  foc_voltage(&m, NAN, 0, 0);
  assert(writes == 2 && stops == 2);
  assert(m.fault.latched & FAULT_PARAM);
  foc_voltage(NULL, 0, 0, 0);
  assert(writes == 2 && stops == 2);
  motor_runtime_t rt = {0};
  m.fault.latched = 0;
  m.out.pwm_enable = 1;
  m.config.pid.id.limit = m.config.pid.iq.limit = 10.0f;
  m.config.pid.iq.kp = 1.0f;
  foc_pid_init(&m.id_pid, 2u);
  foc_pid_init(&m.iq_pid, 2u);
  rt.profile_cst_outer = true;
  m.state.i_bus = 3.0f;
  ticks = UINT32_MAX - 300u;  /* mock PWM application crosses CYCCNT wrap */
#if FOC_REUSE_PARK_SINCOS
  m.state.sin_elec = 1.0f;
  m.state.cos_elec = 0.0f;
#endif
  foc_current(&m, &rt, 2.0f);
  assert(writes == 3 && stops == 2 && rt.foc_cur_loop_us == 4u && limit_reads == 5);
  assert(m.state.v_q == 2.0f && m.state.i_bus == 3.0f);
#if FOC_REUSE_PARK_SINCOS
  assert(m.inv_park_result.u_alpha == -2.0f && m.inv_park_result.u_beta == 0.0f);
#else
  assert(m.inv_park_result.u_alpha == 0.0f && m.inv_park_result.u_beta == 2.0f);
#endif
  puts("PASS: real foc_voltage board capability, output bounds and fault/disarm integration");
  return 0;
}

