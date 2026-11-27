#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "foc_fsm.h"
#include "identification/foc_identification.h"
#include "identification/ident_protocol.h"
#include "foc_modulation.h"
#include "at32_foc_timing.h"
static motor_handle_t motor;
static motor_runtime_t runtime;
static unsigned mask, armed, writes;
static uint32_t now;
static uint8_t response[32];
static bool encoder_cal;
motor_handle_t *motor_get_handler(void) { return &motor; }
motor_runtime_t *motor_runtime(motor_handle_t *m) { return m==&motor?&runtime:NULL; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask=1; }
void __set_PRIMASK(uint32_t x) { mask=x; }
void __DMB(void) { }
uint32_t dwt_get_ticks_us(void) { return now; }
uint32_t dwt_get_ticks(void) { return now*180u; }
uint32_t dwt_ticks_to_us(uint32_t t) { return t/180u; }
void motor_arm(motor_handle_t *m) { assert(!mask); armed=1; m->out.pwm_enable=1; }
void motor_disarm(motor_handle_t *m) { foc_ident_on_disarm(m); armed=0; m->out.pwm_enable=0; }
void motor_shutdown(motor_handle_t *m) { motor_disarm(m); }
bool motor_is_armed(const motor_handle_t *m) { (void)m; return armed!=0; }
void motor_sync_arm_with_mode(motor_handle_t *m) { motor_arm(m); }
void motor_fault_raise(motor_handle_t *m,fault_t f) { m->fault.latched|=f; }
int motor_fault_is_latched(const motor_handle_t *m) { return m->fault.latched!=0; }
error_t foc_pid_clear(pid_state_t *p) { memset(p,0,sizeof(*p));return ERR_NONE; }
bool mc_kth71_cal_active(void) { return encoder_cal; }
void foc_voltage(motor_handle_t *m,float uq,float ud,float angle) {
  foc_modulation_t mod;
  assert(armed && !m->fault.latched);
  assert(foc_modulate_voltage(ud,uq,angle,m->state.vbus_filtered,AT32_FOC_CURRENT_MAX_DUTY,&mod));
  m->out.duty_a=mod.duty_a; m->out.duty_b=mod.duty_b; m->out.duty_c=mod.duty_c; writes++;
}
static void reply(uint8_t cmd,const uint8_t *p,uint8_t n) {
  assert(cmd>=IDENT_CMD_CONTROL && cmd<=IDENT_CMD_APPLY && n==28);
  memcpy(response,p,n);
}
static foc_ident_status_t status(void) { foc_ident_status_t s;foc_ident_status(&s);assert(mask==0);return s; }
static void reset(void) {
  foc_ident_stop(&motor); memset(&motor,0,sizeof(motor));memset(&runtime,0,sizeof(runtime));
  armed=mask=writes=0; now=0xffffff00u;encoder_cal=false;
  motor.config.limits.id_limit=motor.config.limits.iq_limit=2;
  motor.config.limits.sw_ocp=2;
  motor.config.limits.vd_limit=motor.config.limits.vq_limit=5;
  motor.config.param.pole_pairs=5;motor.config.param.direction=DIR_CW;
  motor.config.param.phase_resistance=9;motor.config.param.inductance_d=0.01f;
  motor.config.param.inductance_q=0.02f;motor.config.param.flux_linkage=0.04f;
  motor.state.vbus_filtered=24;motor.state.theta_elec_rad=0.4f;
  motor.current_offset_cal_done=true;runtime.electrical_align_done=true;
}
static void tick(void) { now+=50;foc_fsm_loop(&motor);foc_ident_run_isr(&motor); }
static void lifecycle(void) {
  reset();
  runtime.electrical_align_done=false;
  assert(foc_ident_start(&motor,IDENT_ALL,NULL)==ERR_ALIGIN_FAILED && !armed);
  runtime.electrical_align_done=true; motor.current_offset_cal_done=false;
  assert(foc_ident_start(&motor,IDENT_ALL,NULL)==ERR_OFFSET_CAL);
  motor.current_offset_cal_done=true;
  assert(foc_ident_start(&motor,IDENT_ALL,NULL)==ERR_NONE && armed);
  tick(); assert(writes && motor.fsm==STATE_RSLS_CALIBRATION);
  assert(foc_ident_start(&motor,IDENT_RSLS,NULL)==ERR_STATE);
  assert(foc_ident_apply(&motor)==ERR_STATE);
  foc_ident_stop(&motor);assert(!armed && motor.fsm==STATE_IDLE && status().stage==IDENT_CANCELLED);
  assert(motor.config.param.phase_resistance==9);
  assert(foc_ident_start(&motor,IDENT_ALL,NULL)==ERR_NONE);
  motor_shutdown(&motor); assert(!foc_ident_active(&motor) && status().stage==IDENT_CANCELLED);
  assert(foc_ident_start(&motor,IDENT_ALL,NULL)==ERR_NONE);
  motor.fault.latched=FAULT_GATE_DRIVER;foc_on_fault_latched(&motor);
  assert(!armed && motor.fsm==STATE_FAULT && status().error==IDENT_EXTERNAL);
  reset();assert(foc_ident_start(&motor,IDENT_ALL,NULL)==ERR_NONE);
  tick();now+=500;tick();assert(!armed && status().error==IDENT_TIMING);
  reset();assert(foc_ident_start(&motor,IDENT_RSLS,NULL)==ERR_NONE);
  motor.state.phase_current.ampere[1]=NAN;tick();assert(!armed && status().error==IDENT_NONFINITE);
  reset();assert(foc_ident_start(&motor,IDENT_RSLS,NULL)==ERR_NONE);
  motor.state.vbus_filtered=4;tick();assert(!armed && status().error==IDENT_VOLTAGE);
}
static void identification_and_wire(void) {
  /* v1, RSLS, 300 mA, 3 V, 0.2 V, 200 Hz, 20 rad/s, 1 A, 30 rad/s */
  const uint8_t start[16]={1,1,1,44,11,184,0,200,0,200,7,208,3,232,11,184};
  const uint8_t version=1;
  float id=0,iq=0;
  reset();
  assert(!ident_protocol_handle(&motor,0x90,start,16,reply));
  ident_protocol_handle(&motor,IDENT_CMD_CONTROL,start,1,reply);assert(response[1]==ERR_PARAM&&!armed);
  encoder_cal=true;ident_protocol_handle(&motor,IDENT_CMD_CONTROL,start,16,reply);
  assert(response[1]==ERR_CALIBRATING&&!armed);encoder_cal=false;
  ident_protocol_handle(&motor,IDENT_CMD_CONTROL,start,16,reply);assert(response[1]==0 && armed);
  for(unsigned k=0;k<100000 && foc_ident_active(&motor);k++) {
    float alpha=(2*motor.out.duty_a-motor.out.duty_b-motor.out.duty_c)*8;
    float beta=(motor.out.duty_b-motor.out.duty_c)*24*ONE_DIV_SQRT_3;
    float sn=sinf(0.4f), cs=cosf(0.4f);
    float vd=alpha*cs+beta*sn, vq=-alpha*sn+beta*cs;
    id=vd/1.5f+(id-vd/1.5f)*expf(-1.5f*0.00005f/0.0012f);
    iq=vq/1.5f+(iq-vq/1.5f)*expf(-1.5f*0.00005f/0.0018f);
    motor.state.i_d=id;motor.state.i_q=iq;
    motor.state.phase_current.ampere[0]=sqrtf(id*id+iq*iq);
    tick();
  }
  foc_ident_status_t s=status();
  printf("adapter stage=%d error=%d Rs=%g Ld=%g Lq=%g\n",s.stage,s.error,s.result.rs,s.result.ld,s.result.lq);
  assert(s.stage==IDENT_DONE && !armed && motor.fsm==STATE_IDLE);
  assert(fabsf(s.result.rs-1.5f)<0.02f && fabsf(s.result.ld-0.0012f)<0.00005f);
  assert(motor.config.param.phase_resistance==9);
  motor.config.param.pole_pairs=6;assert(foc_ident_apply(&motor)==ERR_STATE);
  motor.config.param.pole_pairs=5;
  ident_protocol_handle(&motor,IDENT_CMD_APPLY,&version,1,reply);
  assert(response[1]==0 && response[2]==IDENT_DONE && response[4]==1);
  assert(motor.config.param.phase_resistance==s.result.rs && motor.config.param.flux_linkage==0.04f);
  ident_protocol_handle(&motor,IDENT_CMD_STATUS,&version,1,reply);
  uint32_t bits=(uint32_t)response[12]<<24|(uint32_t)response[13]<<16|(uint32_t)response[14]<<8|response[15];
  float rs;memcpy(&rs,&bits,4);assert(rs==s.result.rs);
}
int main(void) { lifecycle();identification_and_wire();puts("PASS identification lifecycle, PWM reconstruction, wire and apply");return 0; }
