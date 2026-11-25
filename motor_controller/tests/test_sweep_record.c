#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sweep_record.h"
void __DMB(void) {}
static unsigned calls, first, count, allow_send;
static uint8_t previous[56], previous_length;
static bool send_frame(uint32_t node, uint8_t cmd,const uint8_t *p,uint8_t len)
{
  assert(node==1 && cmd==0xbf && p[0]==1 && p[2]==0x12 && p[3]==0x34);
  count=p[1];first=((unsigned)p[4]<<24)|((unsigned)p[5]<<16)|((unsigned)p[6]<<8)|p[7];
  assert(count>=1 && count<=6 && len==8+count*8);
  memcpy(previous,p,len);previous_length=len;++calls;return allow_send!=0;
}
int main(void)
{
  unsigned i;uint8_t copy[56];
  sweep_record_begin(0x1234,false);
  sweep_record_push(0,.1f,.2f);sweep_record_end();assert(!sweep_record_poll(1,send_frame));
  sweep_record_begin(0x1234,true);
  for(i=0;i<32;++i)sweep_record_push(i,(float)i,-(float)i); /* 24 fit, 8 dropped */
  assert(!sweep_record_poll(1,send_frame) && first==0 && count==6);
  assert(sweep_record_dropped()==8 && sweep_record_submitted()==0);
  memcpy(copy,previous,previous_length);allow_send=1;
  assert(sweep_record_poll(1,send_frame));assert(!memcmp(copy,previous,previous_length));
  for(i=32;i<38;++i)sweep_record_push(i,(float)i,-(float)i);
  for(i=6;i<24;i+=6){assert(sweep_record_poll(1,send_frame));assert(first==i);}
  assert(sweep_record_poll(1,send_frame)&&first==32);
  assert(!sweep_record_poll(1,send_frame));
  sweep_record_push(38,1,-1);sweep_record_end();
  assert(sweep_record_poll(1,send_frame)&&first==38&&count==1);
  assert(previous[8]==0x3f && previous[9]==0x80 && previous[12]==0xbf);
  assert(sweep_record_dropped()==8 && sweep_record_submitted()==31);
  sweep_record_begin(0x1234,true);
  assert(!sweep_record_dropped() && !sweep_record_submitted());
  for(i=0;i<12000;++i){
    sweep_record_push(i,.125f,-.25f);
    if(i%6==5){assert(sweep_record_poll(1,send_frame));assert(first==i-5);}
  }
  assert(!sweep_record_poll(1,send_frame));assert(calls>2000);
  puts("PASS capture: disabled, bounded overflow, exact gap indices, TX retry, tail, binary32, counter wrap");
  return 0;
}
