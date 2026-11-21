#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sweep/foc_sweep.h"
#include "sweep/sweep_protocol.h"
#include "common.h"
#include "control_mode.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "foc_sense.h"

static motor_handle_t motor;
static motor_runtime_t runtime;
static unsigned armed, disarms, applied, mask, force_limit, work_us;
static unsigned dwt_reads, irq_masks;
static uint32_t ticks;
static float y, last_command;
static uint8_t response[48], response_cmd, response_length;
motor_handle_t *motor_get_handler(void) { return &motor; }
motor_runtime_t *motor_runtime(motor_handle_t *m) { return m == &motor ? &runtime : NULL; }
bool motor_is_armed(const motor_handle_t *m) { return m == &motor && armed != 0u; }
void motor_disarm(motor_handle_t *m) { foc_sweep_on_disarm(m); armed = 0; m->out.pwm_enable = 0; ++disarms; }
void motor_shutdown(motor_handle_t *m) { motor_disarm(m); }
void motor_sync_arm_with_mode(motor_handle_t *m) { armed = 1; m->out.pwm_enable = 1; }
int motor_fault_is_latched(const motor_handle_t *m) { return m->fault.latched != 0; }
void motor_fault_raise(motor_handle_t *m, fault_t f) { m->fault.status |= f; m->fault.latched |= f; }
bool motor_fault_check_shaft_speed(motor_handle_t *m) { return m->fault.latched != 0; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { ++irq_masks; mask = 1; }
void __set_PRIMASK(uint32_t value) { mask = value; }
void __DMB(void) { }
uint32_t dwt_get_ticks(void) { ++dwt_reads; return ticks; }
uint32_t dwt_ticks_to_us(uint32_t value) { return value / 180u; }
uint32_t dwt_elapsed_us(uint32_t a, uint32_t b) { return (b - a) / 180u; }
uint32_t dwt_get_ticks_us(void) { return ticks / 180u; }
error_t foc_pid_clear(pid_state_t *p) { p->last_integral = p->last_output = 0; return ERR_NONE; }
error_t foc_pid_calc(pid_state_t *pid, const pid_param_t *p, float target, float feedback, float *out)
{ *out = pid->last_output = target - feedback; return ERR_NONE; }
uint32_t foc_sense_update_timed(motor_handle_t *m, foc_sense_marks_t *marks, bool outer)
{
  m->state.i_q = y;
  if (marks) memset(marks, 0, sizeof(*marks));
  return 0;
}
static void foc_current(motor_handle_t *m, motor_runtime_t *rt, float q)
{
  m->state.v_q = force_limit == 2 ? 20.0f : (force_limit ? 10.0f : q);
  m->state.v_d = 0.0f;
  if (armed) {
    last_command = q; ++applied;
    y = 0.777767679f * y + (1.0f - 0.777767679f) * q;
  }
  ticks += work_us * 180u;
}
static void foc_speed_open_control(motor_handle_t *m, float target) { (void)m; (void)target; }
#include "sweep_foc.inc"

static void reply(uint8_t cmd, const uint8_t *data, uint8_t length)
{ response_cmd = cmd; response_length = length; memcpy(response, data, length); }
static foc_sweep_status_t status(void)
{ foc_sweep_status_t s; foc_sweep_status(&s); return s; }
static void reset_motor(void)
{
  foc_sweep_stop(&motor, SWEEP_STOP);
  memset(&motor, 0, sizeof(motor)); memset(&runtime, 0, sizeof(runtime));
  motor.ctrl_mode = CTRL_MODE_CST; motor.fsm = STATE_RUNNING;
  motor.current_offset_cal_done = true; motor.out.pwm_enable = 1;
  motor.config.limits.iq_limit = 2;
  motor.config.pid.iq.limit = motor.config.pid.id.limit = 10;
  runtime.current_soft_speed_rad_s = 10;
  armed = 1; disarms = applied = force_limit = mask = work_us = 0;
  dwt_reads = irq_masks = 0;
  y = last_command = 0;
  ticks = 0xffff0000u; /* cross DWT wrap during each run */
}
static void start(void)
{
  sweep_config_t c = {SWEEP_MULTISINE, 8, 20, 2000, .1f, FOC_CURRENT_LOOP_FS_HZ};
  assert(!foc_sweep_start(&motor, &c, 0x1234));
  assert(status().state == SWEEP_RUNNING && mask == 0);
}
static void tick(void) { ticks += 9000u; foc_control_loop(&motor); }
static void aborted(unsigned flags)
{
  foc_sweep_status_t s = status();
  assert(s.state == SWEEP_ABORTED && (s.flags & flags));
  assert(!armed && motor.ref.iq == 0 && mask == 0);
  tick(); assert(!armed); /* no FSM re-arm */
}
int main(void)
{
  unsigned i;
  sweep_result_t r;
  sweep_config_t c = {2, 8, 20, 2000, .1f, FOC_CURRENT_LOOP_FS_HZ};
  uint8_t packet[14] = {1, 1, 0x12, 0x34, 0, 20, 7, 0xd0, 8, 0, 100};
  reset_motor();
  motor.ctrl_mode = CTRL_MODE_SPEED; assert(foc_sweep_start(&motor, &c, 1) == 1);
  motor.ctrl_mode = CTRL_MODE_CST; armed = 0; assert(foc_sweep_start(&motor, &c, 1) == 1);
  armed = 1; motor.ref.iq = .1f; assert(foc_sweep_start(&motor, &c, 1) == 1);
  motor.ref.iq = 0; motor.config.limits.iq_limit = .1f; assert(foc_sweep_start(&motor, &c, 1) == 2);
  motor.config.limits.iq_limit = 2; motor.config.pid.iq.ramp = 1; assert(foc_sweep_start(&motor, &c, 1) == 2);
  motor.config.pid.iq.ramp = 0;
  /* Prepared-but-not-started rejection seen on hardware: retain the exact cause
   * in the start reply, without adding globals or changing ISR guards. */
  motor.state.motor_temp.value = 75.25f;
  assert(sweep_protocol_handle(&motor, 0xbc, packet, sizeof(packet), reply));
  assert(response_length == 48 && response[1] == 1 && response[2] == SWEEP_IDLE);
  assert(response[6] == 8 && status().samples == 0 && !foc_sweep_is_active(&motor));
  assert(response[40] == SWEEP_REJECT_MOTOR_TEMP && response[41] == 0xa7 && response[42] == 1);
  assert(response[44] == 0x42 && response[45] == 0x96 && response[46] == 0x80 && response[47] == 0);
  assert(armed && disarms == 0 && mask == 0); /* reject does not secretly run */
  motor.state.motor_temp.value = 25;
  motor.state.board_temp.value = 61;
  assert(sweep_protocol_handle(&motor, 0xbc, packet, sizeof(packet), reply));
  assert(response[1] == 1 && response[40] == SWEEP_REJECT_BOARD_TEMP);
  motor.state.board_temp.value = 25;
  motor.ref.iq = .2f;
  assert(sweep_protocol_handle(&motor, 0xbc, packet, sizeof(packet), reply));
  assert(response[1] == 1 && response[40] == SWEEP_REJECT_REFERENCE);
  motor.ref.iq = 0;
  motor.state.i_d = NAN;
  assert(sweep_protocol_handle(&motor, 0xbc, packet, sizeof(packet), reply));
  assert(response[1] == 1 && response[40] == SWEEP_REJECT_NONFINITE);
  motor.state.i_d = 0;
  assert(sweep_protocol_handle(&motor, 0xbc, packet, 10, reply)); assert(response[1] == 2);
  assert(sweep_protocol_handle(&motor, 0xbc, packet, sizeof(packet), reply)); /* DLC padding */
  assert(response_length == 40 && response_cmd == 0xbc && !response[1] && response[2] == 1);
  assert(response[27] == 0xa5 && response[38] == 0xa6 && response[39] == 1);
  assert(response[4] == 0x12 && response[5] == 0x34 && response[10] == 0x4e && response[11] == 0x20);
  assert(foc_sweep_start(&motor, &c, 2) == 1);
  assert(foc_sweep_result(0x1234, 0, &r) == 1);
  for (i = 0; i < 4000; ++i) tick();
  assert(status().state == SWEEP_DONE && status().samples == 4000 && status().flags == 0);
  assert(status().interval_min_us == 50 && status().interval_max_us == 50);
  assert(applied == 4000 && disarms == 1 && !armed && motor.fsm == STATE_IDLE);
  assert(!foc_sweep_result(0x1234, 0, &r) && r.coherence > .99f);
  assert(foc_sweep_result(0x1235, 0, &r) == 1);
  packet[0] = 0x12; packet[1] = 0x34; packet[2] = 0;
  assert(sweep_protocol_handle(&motor, 0xbe, packet, 6, reply));
  assert(response_length == 34 && response[1] == 0 && response[5] == SWEEP_DONE && response[33] == 3);
  foc_sweep_stop(&motor, SWEEP_STOP); assert(status().state == SWEEP_DONE);
  assert(sweep_protocol_handle(&motor, 0xbd, NULL, 0, reply) && response[2] == SWEEP_DONE);
  assert(!sweep_protocol_handle(&motor, 0x03, NULL, 0, reply));
  reset_motor(); start(); tick(); motor_disarm(&motor); aborted(SWEEP_STOP);
  reset_motor(); start(); tick(); motor_shutdown(&motor); aborted(SWEEP_STOP);
  reset_motor(); start(); tick(); motor.fault.latched = FAULT_PARAM; tick(); aborted(SWEEP_STOP);
  reset_motor(); start(); tick(); motor.ctrl_mode = CTRL_MODE_SPEED; tick(); aborted(SWEEP_STOP);
  reset_motor(); start(); tick(); foc_sweep_stop(&motor, SWEEP_COMMAND); aborted(SWEEP_COMMAND);
  reset_motor(); start(); tick(); motor.ref.iq = .1f; tick(); aborted(SWEEP_STOP);
  reset_motor(); start(); tick(); motor.state.shaft.position_rad = .51f; tick(); aborted(SWEEP_MOTION);
  reset_motor(); start(); tick(); motor.state.shaft.velocity_rad_s = 5.1f; tick(); aborted(SWEEP_MOTION);
  reset_motor(); start(); tick(); motor.state.board_temp.value = 61; tick(); aborted(SWEEP_LIMIT);
  reset_motor(); start(); tick(); y = NAN; tick(); aborted(SWEEP_NONFINITE);
  reset_motor(); start(); tick(); motor.config.limits.iq_limit = .05f; tick(); aborted(SWEEP_LIMIT);
  reset_motor(); assert(!foc_sweep_start(&motor, &c, 0x1234)); tick();
  runtime.current_soft_speed_rad_s = 4; motor.state.shaft.velocity_rad_s = 4;
  tick(); aborted(SWEEP_LIMIT);
  reset_motor(); start(); tick(); motor.config.pid.iq.limit = 100;
  force_limit = 2; tick(); aborted(SWEEP_LIMIT);
  reset_motor(); start(); tick(); ticks += 9000; tick(); aborted(SWEEP_TIMING);
  reset_motor(); start(); work_us = 46; tick(); aborted(SWEEP_TIMING);
  assert(status().timing_reason == 2 && status().max_us == 46 && status().failure_sample == 1);
  /* Last sample limit must invalidate completion, not publish a valid response. */
  reset_motor(); start(); for (i = 0; i < 3999; ++i) tick(); force_limit = 1; tick(); aborted(SWEEP_LIMIT);
  assert(foc_sweep_result(0x1234, 0, &r) == 1);
  reset_motor(); motor.ref.iq = .3f; tick(); assert(last_command == .3f && armed); /* transparent idle */
  assert(dwt_reads == 2 && irq_masks == 0);
  foc_sweep_stop(&motor, SWEEP_STOP); foc_sweep_on_disarm(&motor);
  assert(dwt_reads == 2 && irq_masks == 0 && armed && motor.ref.iq == .3f);
  /* Inactive diagnostic build: normal reference and no sweep timer reads/IRQ work. */
  assert(mask == 0);
  puts("PASS real FOC dispatch, guard/limits, cancellation, final-sample rejection, wrap, wire v1, session IDs and passive status");
  return 0;
}
