#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "encoder.h"
#include "foc_sense.h"
#include "encoder/kth71xx.h"
#include "motor.h"
static motor_handle_t motor;
static motor_runtime_t runtime;
static uint16_t raw_counts;
static unsigned crc_ok=1;
static uint32_t time_us;
float s_inv_gear_ratio=1;
uint32_t board_time_us(void) { return (uint16_t)(time_us+=50); }
uint32_t dwt_get_ticks(void) { return time_us*180; }
uint32_t dwt_get_ticks_us(void) { return time_us; }
uint32_t dwt_ticks_to_us(uint32_t ticks) { return ticks/180; }
uint32_t dwt_elapsed_us(uint32_t a,uint32_t b) { return (b-a)/180; }
motor_runtime_t *motor_runtime(motor_handle_t *m) { return m==&motor?&runtime:NULL; }
void KTH71_InitCS(kth71_cs_t cs) { assert(cs==KTH71_CS_0); }
unsigned char KTH71_ReadAngleCS(kth71_cs_t cs,uint16_t *raw) { assert(cs==KTH71_CS_0);*raw=raw_counts;return (unsigned char)crc_ok; }
int main(void)
{
  unsigned i,p;int dir;
  for(dir=-1;dir<=1;dir+=2) for(p=1;p<=16;++p)
  {
    memset(&motor,0,sizeof(motor));encoder_init();
    motor.config.param.direction=dir; motor.config.param.pole_pairs=p;
    motor.config.param.theta_elec_offset_rad=.73f;
    for(i=0;i<8192;++i)
    {
      double expected=(double)dir*i*(double)_2PI/8192*p-.73f;
      raw_counts=(uint16_t)(i<<3);encoder_update(&motor,false);
      assert(motor.state.theta_elec_rad>=0 && motor.state.theta_elec_rad<_2PI);
      assert(fabs(sin(expected)-sin(motor.state.theta_elec_rad))<2e-5);
      assert(motor.state.shaft.counts==i);
    }
    {
      float old=motor.state.theta_elec_rad;
      crc_ok=0;raw_counts=0xffff;encoder_update(&motor,false);
      assert(motor.state.theta_elec_rad==old && motor.state.shaft.counts==8191);
      crc_ok=1;raw_counts=0;encoder_update(&motor,false);
      assert(motor.state.shaft.counts==0);
    }
  }
  puts("PASS actual encoder: 262144 angles, both directions, 1..16 pole pairs, wrap and failed-CRC hold/recovery");
  return 0;
}
