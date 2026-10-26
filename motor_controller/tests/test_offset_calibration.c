#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "current_sense.h"
#include "motor.h"
static motor_runtime_t rt;
static motor_handle_t motor;
static unsigned reads,fail_at,enters,disarms,ends;
static error_t begin_result;
motor_runtime_t *motor_runtime(motor_handle_t *m) { return m == &motor ? &rt : NULL; }
void motor_enter_dc_cal_hw(motor_handle_t *m) { assert(m == &motor); ++enters; }
void motor_disarm(motor_handle_t *m) { assert(m == &motor); ++disarms; }
void motor_fault_raise(motor_handle_t *m, fault_t bits) { m->fault.latched |= bits; m->fault.status |= bits; }
error_t current_hw_calibration_begin(uint32_t *state) { *state=7; return begin_result; }
void current_hw_calibration_end(uint32_t state) { assert(state == 7); assert(disarms); ++ends; }
error_t current_hw_calibration_read(motor_handle_t *m) {
  ++reads;
  if (reads == fail_at) return ERR_TIMEOUT;
  m->state.phase_current.adc_raw[0]=(uint16_t)(2000+2*(reads&1));
  m->state.phase_current.adc_raw[1]=(uint16_t)(2100+2*(reads&1));
  m->state.phase_current.adc_raw[2]=(uint16_t)(2200+2*(reads&1));
  return ERR_NONE;
}
static void reset(void) { memset(&motor,0,sizeof motor); memset(&rt,0,sizeof rt); reads=fail_at=enters=disarms=ends=0; begin_result=ERR_NONE; }
int main(void) {
  unsigned n;
  reset(); current_sense_init(&motor);
  assert(reads == CURRENT_OFFSET_SETTLING_SAMPLES + CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT);
  assert(motor.current_offset_cal_done && !motor.current_offset_calibrating && !motor.current_offset_cal_pending);
  assert(rt.offset_cnt == CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT);
  assert(motor.state.phase_current.adc_offset[0]==2001 && motor.state.phase_current.adc_offset[1]==2101 && motor.state.phase_current.adc_offset[2]==2201);
  assert(enters==1 && disarms==1 && ends==1 && !motor.fault.latched);
  motor.current_offset_cal_pending=true;
  for(n=0;n<1000;++n) current_sense_process_sample(&motor);
  assert(rt.offset_cnt == CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT && reads == CURRENT_OFFSET_SETTLING_SAMPLES + CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT);
  reset(); fail_at=5; current_sense_init(&motor);
  assert(!motor.current_offset_cal_done && !motor.current_offset_calibrating && !motor.current_offset_cal_pending);
  assert(rt.offset_cnt==0 && ends==1 && disarms==1 && (motor.fault.latched & FAULT_CURRENT_SENSOR));
  reset(); fail_at=CURRENT_OFFSET_SETTLING_SAMPLES+34; current_sense_init(&motor);
  assert(rt.offset_cnt==33 && !motor.current_offset_cal_done && !rt.offset_active);
  assert(motor.state.phase_current.adc_offset[0]==0 && ends==1 && disarms==1);
  reset(); begin_result=ERR_STATE; current_sense_init(&motor);
  assert(reads==0 && enters==0 && ends==0 && disarms==1 && !motor.current_offset_cal_done);
  assert(!motor.current_offset_cal_pending && !motor.current_offset_calibrating && !rt.offset_active);
  current_sense_init(NULL);
  puts("PASS S03: 65536 fresh samples, settling discard, exact mean, no repeated-read accumulation, failures and cleanup");
  return 0;
}
