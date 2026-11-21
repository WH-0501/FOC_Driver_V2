#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "control_mode.h"
#include "motor.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "common.h"
/* This suite checks the original control behavior with the optional feature
 * compiled out. run_sweep_tests.py covers the enabled real FOC dispatch. */
#define FOC_SWEEP_ENABLED 0
#include "sweep/foc_sweep.h"

static motor_handle_t motor;
static motor_runtime_t runtime;
static unsigned armed,disarms,clears,sensed,executed;
static float applied_iq;
static unsigned reply_result,reply_len;
typedef struct { void *gate; bool armed; } motor_driver_t;
static unsigned pwm_stops,phase_off;
error_t pwm_hw_stop(void) { ++pwm_stops;return ERR_NONE; }
void gate_driver_set_all_phase_en(void *gate,bool enable) { assert(gate && !enable);++phase_off; }
void gate_driver_exit_sleep(void *gate) { (void)gate;assert(0); }
motor_handle_t *motor_get_handler(void) { return &motor; }
motor_runtime_t *motor_runtime(motor_handle_t *m) { return m==&motor ? &runtime:NULL; }
bool motor_is_armed(const motor_handle_t *m) { (void)m;return armed!=0; }
void motor_disarm(motor_handle_t *m) { ++disarms;armed=0;m->out.pwm_enable=0; }
void motor_shutdown(motor_handle_t *m) { motor_disarm(m); }
void motor_sync_arm_with_mode(motor_handle_t *m) { armed=control_mode_needs_pwm(m->ctrl_mode); }
int motor_fault_is_latched(const motor_handle_t *m) { return m->fault.latched!=0; }
void motor_fault_raise(motor_handle_t *m,fault_t bits) { m->fault.latched|=bits; }
error_t foc_pid_clear(pid_state_t *p) { ++clears;p->last_integral=0;return ERR_NONE; }
#include "foc_sense.h"
static uint32_t test_ticks;
static bool sensed_outer;
static unsigned position_calls, speed_calls, current_calls;
uint32_t dwt_get_ticks(void) { test_ticks += 180u; return test_ticks; }
uint32_t dwt_ticks_to_us(uint32_t ticks) { return ticks / 180u; }
uint32_t dwt_elapsed_us(uint32_t start_ticks, uint32_t end_ticks) { return (end_ticks - start_ticks) / 180u; }
void foc_sense_update(motor_handle_t *m) {
  (void)m; ++sensed; test_ticks += 360u;
  if (runtime.profile_cst_outer) runtime.cst_shaft_spi_ticks = 360u;
}
uint32_t foc_sense_update_timed(motor_handle_t *m, foc_sense_marks_t *marks, bool outer_tick) {
  sensed_outer=outer_tick;
  if (!marks) {
    uint32_t start = test_ticks;
    foc_sense_update(m);
    test_ticks += 100u;
    runtime.foc_sense_update_us = dwt_elapsed_us(start, test_ticks);
    return test_ticks - start;
  }
  ++sensed;
  test_ticks += 2u * 180u;
  marks->current_end = test_ticks;
  runtime.cst_shaft_spi_ticks = 5u * 180u;
  test_ticks += 9u * 180u;
  marks->position_end = test_ticks;
  test_ticks += 3u * 180u;
  marks->encoder_end = test_ticks;
  test_ticks += 180u;
  test_ticks += 2u * 180u;
  return 0u;
}
static void check_sense_fields(void) {
  assert(runtime.foc_sense_update_us == 18u && runtime.foc_current_sense_us == 2u);
  assert(runtime.foc_encoder_update_us == 5u && runtime.foc_vel_loop_us == 3u);
  assert(runtime.foc_pos_loop_us == 4u);
}
void foc_current(motor_handle_t *m,motor_runtime_t *rt,float q) { ++current_calls;applied_iq=q;executed=m->ctrl_mode; test_ticks += 820u; rt->foc_cur_loop_us=4u;
}
static error_t foc_pid_calc(pid_state_t *pid,const pid_param_t *param,float target,float feedback,float *out) {
  (void)param; assert(sensed_outer);
  if (pid==&motor.position_pid) ++position_calls;
  else { assert(pid==&motor.velocity_pid); ++speed_calls; }
  *out=pid->last_output=target-feedback;
  return ERR_NONE;
}
static void foc_speed_open_control(motor_handle_t *m,float v) { (void)m;(void)v;executed=CTRL_MODE_SPEED_OPEN_LOOP; }
error_t mc_disable(void) { motor_disarm(&motor);foc_fsm_next_state(&motor,STATE_IDLE);return ERR_NONE; }
static void can_fd_custom_reply(unsigned cmd,const uint8_t *payload,unsigned len) { (void)cmd;reply_result=payload[0];reply_len=len; }
#define CAN_FD_CMD_CONTROL_MODE 0x41
#define CAN_FD_CMD_ENABLE 0x40
/* Exact API/dispatch/loop functions and protocol case extracted by the runner. */
#include "c06_impl.inc"
static void reset(void) {
  memset(&motor,0,sizeof motor);memset(&runtime,0,sizeof runtime);
  runtime.shaft_hard_overspeed_enabled=true;
  motor.current_offset_cal_done=true;motor.ctrl_mode=CTRL_MODE_SPEED;motor.fsm=STATE_IDLE;
  armed=disarms=clears=sensed=executed=0;reply_result=999;reply_len=0;
}
static void check_outer_ticks(control_mode_t mode) {
  reset(); motor.ctrl_mode=mode; assert(mc_enable()==ERR_NONE);
  runtime.outer_loop_skip=FW_FOC_OUTER_LOOP_DIV-1u; /* simulate idle phase before enable */
  position_calls=speed_calls=current_calls=0;
  float held=0;
  for (unsigned i=0;i<=2u*FW_FOC_OUTER_LOOP_DIV;++i) {
    motor.ref.speed_rad_s=motor.ref.position_rad=1.0f+i;
    motor.state.joint.position_rad=0.25f;
    motor.state.joint.velocity_rad_s=0.5f;
    bool due=i%FW_FOC_OUTER_LOOP_DIV==0u;
    if (due) held=1.0f+i-0.5f-(mode==CTRL_MODE_POSITION ? 0.25f : 0.0f);
    foc_control_loop(&motor);
    assert(sensed_outer==due && applied_iq==held && current_calls==i+1u);
    assert(speed_calls==i/FW_FOC_OUTER_LOOP_DIV+1u);
    assert(position_calls==(mode==CTRL_MODE_POSITION ? speed_calls : 0u));
  }
  assert(mc_disable()==ERR_NONE && mc_enable()==ERR_NONE);
  motor.ref.speed_rad_s=motor.ref.position_rad=-4.0f;
  foc_control_loop(&motor);
  assert(sensed_outer && applied_iq<0);
}
static bool expected(int mode) { return mode==CTRL_MODE_IDLE || mode==CTRL_MODE_TORQUE || mode==CTRL_MODE_CST || mode==CTRL_MODE_SPEED || mode==CTRL_MODE_POSITION || mode==CTRL_MODE_SPEED_OPEN_LOOP; }
int main(void) {
  int mode; uint8_t wire;
  check_outer_ticks(CTRL_MODE_SPEED); check_outer_ticks(CTRL_MODE_POSITION);
  for(mode=-2;mode<=255;++mode) {
    reset();
    assert(control_mode_is_supported((control_mode_t)mode)==expected(mode));
    assert(control_mode_needs_pwm((control_mode_t)mode)==(expected(mode)&&mode!=CTRL_MODE_IDLE));
    assert(mc_set_ctrl_mode((control_mode_t)mode)==(expected(mode)?ERR_NONE:ERR_CTRL_MODE));
    if(!expected(mode)) assert(motor.ctrl_mode==CTRL_MODE_SPEED && clears==0);
  }
  reset(); motor.fsm=STATE_RUNNING;
  assert(mc_set_ctrl_mode(CTRL_MODE_POSITION)==ERR_STATE && motor.ctrl_mode==CTRL_MODE_SPEED);
  reset(); armed=1; assert(mc_set_ctrl_mode(CTRL_MODE_TORQUE)==ERR_STATE);
  reset(); motor.fsm=STATE_ELECTRICAL_ALIGNMENT; assert(mc_set_ctrl_mode(CTRL_MODE_SPEED)==ERR_CALIBRATING);
  reset(); motor.fault.latched=FAULT_PARAM;assert(mc_set_ctrl_mode(CTRL_MODE_SPEED)==ERR_FAULT_LATCHED);
  reset(); assert(mc_set_ctrl_mode(CTRL_MODE_POSITION)==ERR_NONE && clears==4);
  reset(); motor.fsm=STATE_RUNNING;armed=1;assert(mc_set_ctrl_mode(CTRL_MODE_IDLE)==ERR_NONE && !armed && motor.fsm==STATE_IDLE);
  for(mode=1;mode<=255;++mode) {
    reset();motor.ctrl_mode=(control_mode_t)mode;
    { error_t result=mc_enable(); if(result!=(expected(mode)?ERR_NONE:ERR_CTRL_MODE)) { printf("mode=%d result=%d state=%d done=%d\n",mode,result,motor.fsm,motor.current_offset_cal_done);fflush(stdout); } assert(result==(expected(mode)?ERR_NONE:ERR_CTRL_MODE)); }
    if(!expected(mode)) {foc_fsm_request_running(&motor);assert(motor.fsm==STATE_IDLE);}
  }
  for(mode=0;mode<=255;++mode) {
    reset();motor.ctrl_mode=(control_mode_t)mode;motor.fsm=STATE_RUNNING;armed=1;
    foc_control_loop(&motor);
    if(!expected(mode)) assert(!armed && motor.fsm==STATE_FAULT && (motor.fault.latched&FAULT_PARAM) && !sensed && !executed);
    else if(mode==CTRL_MODE_IDLE) assert(!armed && motor.fsm==STATE_IDLE && !executed);
    else assert(executed==(unsigned)mode && sensed==1);
  }
  reset();motor.ctrl_mode=CTRL_MODE_MIT;motor.fsm=STATE_RUNNING;armed=1;
  foc_fsm_loop(&motor);assert(!armed && motor.fsm==STATE_FAULT && motor.fault.latched);
  for(mode=0;mode<=255;++mode) {
    reset();wire=(uint8_t)mode;test_protocol_mode(&wire,1);
    assert(reply_len==1 && reply_result==(unsigned)(expected(mode)?ERR_NONE:ERR_CTRL_MODE));
  }
  reset(); wire=1; test_protocol_enable(&wire,1); assert(reply_len==1 && reply_result==ERR_NONE);
  reset(); wire=1; motor.fault.latched=FAULT_PARAM; test_protocol_enable(&wire,1); assert(reply_result==ERR_FAULT_LATCHED);
  reset(); wire=2; test_protocol_enable(&wire,1); assert(reply_result==ERR_PARAM);
  reset(); test_protocol_enable(NULL,0); assert(reply_result==ERR_PARAM);
  reset(); wire=0; armed=1; test_protocol_enable(&wire,1); assert(reply_result==ERR_NONE && !armed);
  for (mode=0;mode<=1000;++mode) {
    float speed=(float)mode/10.0f;
    float q=speed_limit_soft_iq(1.0f,speed,100.0f);
    assert(q>=0.0f && q<=1.00001f);
    assert(speed_limit_soft_iq(-1.0f,-speed,100.0f)>=-1.00001f);
  }
  reset();test_protocol_mode(NULL,0);assert(reply_result==ERR_PARAM);
  reset();motor.fsm=STATE_RUNNING;wire=CTRL_MODE_POSITION;test_protocol_mode(&wire,1);assert(reply_result==ERR_STATE);
  { motor_driver_t driver={(void*)1,true};motor_driver_disarm(&driver);assert(!driver.armed && pwm_stops==1 && phase_off==1); }
  /* Actual control-loop cutoff must precede all supported mode dispatch. */
  for(mode=1;mode<CTRL_MODE_RESERVED;++mode) if(expected(mode)) {
    int sign;
    for(sign=-1;sign<=1;sign+=2) {
      reset(); motor.ctrl_mode=(control_mode_t)mode;motor.fsm=STATE_RUNNING;armed=1;
      motor.state.shaft.velocity_rad_s=sign*MAX_VELOCITY_RAD_S;
      foc_control_loop(&motor);
      assert(!armed && !executed && motor.fsm==STATE_FAULT);
      if (mode == CTRL_MODE_CST && FOC_PROFILE_CST_OUTER == 1)
      {
        assert(!runtime.profile_cst_outer && runtime.foc_control_loop_us == 20u && runtime.foc_ctrl_loop_us == 0u && runtime.foc_cur_loop_us == 0u);
        check_sense_fields();
      }
      assert(motor.fault.latched & FAULT_OVER_SPEED);
      assert(mc_enable()==ERR_FAULT_LATCHED);
    }
  }
  reset();motor.fsm=STATE_RUNNING;armed=1;
  motor.state.shaft.velocity_rad_s=MAX_VELOCITY_RAD_S-1.0f;
  foc_control_loop(&motor);assert(armed && executed==CTRL_MODE_SPEED && !motor.fault.latched);
  reset();motor.ctrl_mode=CTRL_MODE_TORQUE;motor.fsm=STATE_RUNNING;armed=1;
  runtime.current_soft_speed_rad_s=100.0f;motor.ref.iq=2.0f;motor.config.limits.iq_limit=1.0f;
  motor.state.shaft.velocity_rad_s=95.0f;foc_control_loop(&motor);
  assert(fabsf(applied_iq-0.5f)<0.0001f && runtime.current_speed_limited);
  motor.state.shaft.velocity_rad_s=100.0f;foc_control_loop(&motor);assert(applied_iq==0);
  motor.ref.iq=-2.0f;foc_control_loop(&motor);assert(applied_iq==-1.0f && !runtime.current_speed_limited);
  motor.state.shaft.velocity_rad_s=-95.0f;foc_control_loop(&motor);assert(fabsf(applied_iq+0.5f)<0.0001f);
  reset();motor.ctrl_mode=CTRL_MODE_TORQUE;motor.fsm=STATE_RUNNING;armed=1;
  runtime.shaft_hard_overspeed_enabled=false;
  motor.state.shaft.velocity_rad_s=-MAX_VELOCITY_RAD_S;foc_control_loop(&motor);
  assert(armed && !(motor.fault.latched&FAULT_OVER_SPEED));
  runtime.shaft_hard_overspeed_enabled=true;
  motor.state.shaft.velocity_rad_s=-MAX_VELOCITY_RAD_S;foc_control_loop(&motor);
  assert(!armed && (motor.fault.latched&FAULT_OVER_SPEED));
  /* CST: latest target, held between updates; identical limits, no outer PID. */
  reset();motor.ctrl_mode=CTRL_MODE_CST;motor.fsm=STATE_RUNNING;armed=1;
  runtime.current_soft_speed_rad_s=100.0f;motor.config.limits.iq_limit=1.0f;
  test_ticks = 0xFFFFFF00u; /* The stage sequence crosses the DWT wrap. */
  can_fd_custom_apply_curr(&motor,200);foc_control_loop(&motor);
  assert(executed==CTRL_MODE_CST && applied_iq==.2f);
  assert(!runtime.profile_cst_outer);
  if (FOC_PROFILE_CST_OUTER == 1)
  {
    check_sense_fields();
    assert(runtime.foc_control_loop_us == 26u && runtime.foc_ctrl_loop_us == 5u && runtime.foc_cur_loop_us == 4u);
    motor.state.shaft.velocity_rad_s=MAX_VELOCITY_RAD_S;
    foc_control_loop(&motor);
    assert(motor.fsm == STATE_FAULT);
    check_sense_fields();
    reset(); motor.ctrl_mode=CTRL_MODE_CST; motor.fsm=STATE_RUNNING; armed=1;
    runtime.current_soft_speed_rad_s=100.0f; motor.config.limits.iq_limit=1.0f;
  }
  else
  {
    /* Profiling disabled in current firmware: no control-stage clock reads. */
    assert(runtime.foc_ctrl_loop_us == 0u && runtime.foc_control_loop_us == 8u);
    assert(runtime.foc_sense_update_us == 2u && runtime.foc_vel_loop_us == 0u);
    assert(sensed_outer && runtime.foc_pos_loop_us == 0u);
    foc_control_loop(&motor);
    assert(runtime.foc_pos_loop_us == 0u && sensed_outer == (FW_FOC_OUTER_LOOP_DIV == 1u));
    assert(runtime.foc_vel_loop_us == 0u);
  }
  can_fd_custom_apply_curr(&motor,-300);foc_control_loop(&motor);assert(applied_iq==-.3f);
  foc_control_loop(&motor);assert(applied_iq==-.3f);
  motor.ref.iq=2.0f;motor.state.shaft.velocity_rad_s=95.0f;foc_control_loop(&motor);
  assert(fabsf(applied_iq-.5f)<.0001f && runtime.current_speed_limited && motor.ref.iq==2.0f);
  motor.ref.iq=-2.0f;foc_control_loop(&motor);assert(applied_iq==-1.0f && !runtime.current_speed_limited);
  assert(mc_set_ctrl_mode(CTRL_MODE_TORQUE)==ERR_STATE);
  assert(mc_disable()==ERR_NONE && !armed);
  assert(mc_set_ctrl_mode(CTRL_MODE_TORQUE)==ERR_NONE);
  puts("PASS CST: latest/hold/reverse, current clamp, soft limit, braking, preserved reference and mode switch guard");
  puts("PASS speed limits: bidirectional hard cutoff, all modes, latch, soft taper, braking and disabled soft");
  puts("PASS C06: exhaustive mode admission/enable/dispatch, active/calibration/fault guards, PID reset, stale-output shutdown and CAN status");
  return 0;
}
