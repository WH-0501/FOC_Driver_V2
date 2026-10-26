#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "error_types.h"
typedef struct { unsigned snapshot; } motor_handle_t;
typedef struct { struct { uint32_t pcceien; } ctrl1_bit; } adc_type;
static adc_type adc;
#define ADC2 (&adc)
#define ADC_PCCE_INT 128
#define ADC_PCCE_FLAG 4
#define TRUE 1
#define FALSE 0
#define RESET 0
static unsigned flag,clock_us,events,irq_mask,ipsr,read_count;
static int generate=1,advance_clock=1;
static unsigned __get_IPSR(void) { return ipsr; }
static unsigned __get_PRIMASK(void) { return irq_mask; }
static void __disable_irq(void) { irq_mask=1; }
static void __enable_irq(void) { irq_mask=0; }
static uint32_t board_time_us(void) { if(advance_clock) ++clock_us; return clock_us & 65535; }
static void adc_flag_clear(adc_type *a,unsigned f) { (void)a;(void)f;flag=0; }
static unsigned adc_flag_get(adc_type *a,unsigned f) { (void)a;(void)f; if(generate && clock_us%50==0) {flag=1;++events;} return flag; }
static void adc_interrupt_enable(adc_type *a,unsigned f,unsigned value) { (void)f;a->ctrl1_bit.pcceien=value; }
static void board_get_phase_current(motor_handle_t *m) { assert(irq_mask); m->snapshot=events; ++read_count; }
/* Include the exact Board implementation extracted by the test runner. */
#include "offset_adc_impl.inc"
int main(void) {
  uint32_t state; motor_handle_t m={0}; unsigned old;
  adc.ctrl1_bit.pcceien=1;flag=1;
  assert(current_hw_calibration_begin(&state)==ERR_NONE && state==1 && !adc.ctrl1_bit.pcceien && !flag);
  assert(current_hw_calibration_read(&m)==ERR_NONE && m.snapshot==1 && read_count==1 && irq_mask==0);
  old=m.snapshot; flag=1;
  assert(current_hw_calibration_read(&m)==ERR_NONE && m.snapshot>old);
  current_hw_calibration_end(state); assert(adc.ctrl1_bit.pcceien==1 && !flag);
  adc.ctrl1_bit.pcceien=0; assert(current_hw_calibration_begin(&state)==ERR_NONE && state==0);
  irq_mask=1; assert(current_hw_calibration_read(&m)==ERR_NONE && irq_mask==1); irq_mask=0;
  current_hw_calibration_end(state); assert(adc.ctrl1_bit.pcceien==0);
  generate=0; clock_us=65530; old=read_count;
  assert(current_hw_calibration_read(&m)==ERR_TIMEOUT && read_count==old);
  assert((uint16_t)(clock_us-65530)>=1000);
  advance_clock=0; assert(current_hw_calibration_read(&m)==ERR_TIMEOUT);
  ipsr=1; assert(current_hw_calibration_begin(&state)==ERR_STATE);
  assert(current_hw_calibration_begin(NULL)==ERR_NULL && current_hw_calibration_read(NULL)==ERR_NULL);
  puts("PASS S03 ADC: stale event discarded, fresh snapshots, IRQ ownership restored, timer wrap, ADC/timebase stopped, ISR rejection");
  return 0;
}
