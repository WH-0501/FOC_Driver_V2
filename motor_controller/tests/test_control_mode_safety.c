#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "control_mode.h"
#include "fra.h"
#include "motor.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "common.h"

static motor_handle_t motor;
static motor_runtime_t runtime;
static motor_handle_t *s_motor=&motor;
static uint8_t fra_reply[64];
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t v){(void)v;}
static uint16_t can_fd_codec_read_u16_be(const uint8_t *p){return (uint16_t)((p[0]<<8)|p[1]);}
static void can_fd_codec_write_u16_be(uint8_t *p,uint16_t v){p[0]=(uint8_t)(v>>8);p[1]=(uint8_t)v;}
static void can_fd_write_i32_be(uint8_t *p,int32_t v){p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;}

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
static uint32_t board_time_us(void){static uint32_t t;static unsigned phase;uint32_t now=t;t+=(phase++&1)?40:10;return now;}
uint32_t dwt_get_ticks_us(void) { return 100; }
static uint32_t foc_dwt_elapsed_us(uint32_t t) { (void)t;return 1; }
void foc_sense_update(motor_handle_t *m) { (void)m;++sensed; }
void foc_current(motor_handle_t *m,float d,float q,float a,float w) { (void)m;(void)d;applied_iq=q;(void)a;(void)w;executed=m->ctrl_mode; }
static void foc_speed_closed_control(motor_handle_t *m,float v) { (void)m;(void)v;executed=CTRL_MODE_SPEED; }
static void foc_position_closed_control(motor_handle_t *m,float p) { (void)m;(void)p;executed=CTRL_MODE_POSITION; }
static void foc_speed_open_control(motor_handle_t *m,float v) { (void)m;(void)v;executed=CTRL_MODE_SPEED_OPEN_LOOP; }
error_t mc_disable(void) { motor_disarm(&motor);foc_fsm_next_state(&motor,STATE_IDLE);return ERR_NONE; }
static void can_fd_custom_reply(unsigned cmd,const uint8_t *payload,unsigned len) { (void)cmd;reply_result=payload[0];reply_len=len;memcpy(fra_reply,payload,len); }
#define CAN_FD_CMD_CONTROL_MODE 0x41
#define CAN_FD_CMD_ENABLE 0x40
#define CAN_FD_CMD_FRA_START 0xBC
/* Exact API/dispatch/loop functions and protocol case extracted by the runner. */
#include "c06_impl.inc"
static void reset(void) {
  memset(&motor,0,sizeof motor);memset(&runtime,0,sizeof runtime);
  runtime.shaft_hard_overspeed_enabled=true;
  motor.current_offset_cal_done=true;motor.ctrl_mode=CTRL_MODE_SPEED;motor.fsm=STATE_IDLE;
  armed=disarms=clears=sensed=executed=0;reply_result=999;reply_len=0;
}
static bool expected(int mode) { return mode==CTRL_MODE_IDLE || mode==CTRL_MODE_TORQUE || mode==CTRL_MODE_CST || mode==CTRL_MODE_SPEED || mode==CTRL_MODE_POSITION || mode==CTRL_MODE_SPEED_OPEN_LOOP; }
int main(void) {
  int mode; uint8_t wire;
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
  motor.ref.iq=.2f;foc_control_loop(&motor);
  assert(executed==CTRL_MODE_CST && applied_iq==.2f);
  motor.ref.iq=-.3f;foc_control_loop(&motor);assert(applied_iq==-.3f);
  foc_control_loop(&motor);assert(applied_iq==-.3f);
  motor.ref.iq=2.0f;motor.state.shaft.velocity_rad_s=95.0f;foc_control_loop(&motor);
  assert(fabsf(applied_iq-.5f)<.0001f && runtime.current_speed_limited && motor.ref.iq==2.0f);
  motor.ref.iq=-2.0f;foc_control_loop(&motor);assert(applied_iq==-1.0f && !runtime.current_speed_limited);
  assert(mc_set_ctrl_mode(CTRL_MODE_TORQUE)==ERR_STATE);
  assert(mc_disable()==ERR_NONE && !armed);
  assert(mc_set_ctrl_mode(CTRL_MODE_TORQUE)==ERR_NONE);
  { uint8_t start[14]={1,1,0,42,0,20,7,208,16,0,100,0,0,0};
    reset();can_fd_fra_command(CAN_FD_CMD_FRA_START,start,sizeof(start));assert(fra_reply[1]!=0);
    motor.ctrl_mode=CTRL_MODE_CST;motor.fsm=STATE_RUNNING;armed=1;motor.config.limits.iq_limit=1;motor.config.pid.iq.limit=10;
    can_fd_fra_command(CAN_FD_CMD_FRA_START,start,sizeof(start));assert(fra_reply[1]==0 && fra_state()==1 && fra_id()==42);
    foc_control_loop(&motor);assert(fra_samples()==1);
    can_fd_custom_apply_curr(&motor,200);assert(fra_state()==3 && (fra_flags()&8));
    foc_control_loop(&motor);assert(!armed && !fra_owned() && motor.ref.iq==0);
    start[1]=0;can_fd_fra_command(CAN_FD_CMD_FRA_START,start,sizeof(start));assert(!armed);
  }
  puts("PASS FRA protocol: padded start, state guard, ISR injection, competing target cancellation and stop");
  puts("PASS CST: latest/hold/reverse, current clamp, soft limit, braking, preserved reference and mode switch guard");
  puts("PASS speed limits: bidirectional hard cutoff, all modes, latch, soft taper, braking and disabled soft");
  puts("PASS C06: exhaustive mode admission/enable/dispatch, active/calibration/fault guards, PID reset, stale-output shutdown and CAN status");
  return 0;
}
