#include "sweep/sweep_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#if FOC_SWEEP_ENABLED
#error This test must compile with the production default (feature off).
#endif
static uint8_t response[34], length;
static void reply(uint8_t cmd, const uint8_t *p, uint8_t n)
{ (void)cmd; length=n; memcpy(response,p,n); }
int main(void)
{
  uint8_t start[11]={1,1,0x12,0x34,0,20,7,0xd0,16,0,50};
  unsigned side_effect=0;
  assert(!foc_sweep_is_active(++side_effect));
  foc_sweep_run_isr(++side_effect, ++side_effect, ++side_effect, ++side_effect);
  foc_sweep_stop(++side_effect, ++side_effect);
  foc_sweep_on_disarm(++side_effect);
  assert(side_effect==0);
  assert(sweep_protocol_handle(NULL,0xbc,start,11,reply));
  assert(length==20 && response[0]==1 && response[1]==3 && response[2]==0);
  assert(response[4]==0x12 && response[5]==0x34);
  assert(sweep_protocol_handle(NULL,0xbd,NULL,0,reply) && response[1]==3);
  assert(sweep_protocol_handle(NULL,0xbe,start+2,3,reply) && length==34 && response[1]==3);
  assert(!sweep_protocol_handle(NULL,0x03,start,11,reply));
  puts("PASS default off: no algorithm linked, no hook side effects, explicit unsupported reply");
  return 0;
}
