#include "sweep_engine.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
float reference_sweep_engine_step(sweep_engine_t *,float);
int main(void){
 unsigned c,f,m,i,n=0;
 for(m=1;m<=3;++m)for(c=4;c<=16;c+=6)for(f=10000;f<=40000;f*=2){
  sweep_engine_t a={0},b;float y=0;
  sweep_config_t cfg={m,c,20,f==10000?1000.0f:2000.0f,.05f,(float)f};
  assert(!sweep_engine_prepare(&a,&cfg));sweep_engine_start(&a);b=a;
  while(a.state==SWEEP_RUNNING){
   float x=sweep_engine_step(&a,y),z=reference_sweep_engine_step(&b,y);
   assert(!memcmp(&x,&z,sizeof(x)));
   assert(a.samples==b.samples&&a.state==b.state&&a.blocks==b.blocks);
   if(a.remaining==0)for(i=0;i<c;++i)assert(!memcmp(&a.tone[i],&b.tone[i],sizeof(a.tone[i])));
   y=.73f*y+.27f*x;++n;
  }
  assert(!memcmp(&a,&b,sizeof(a)));
 }
 printf("PASS old/new engine: %u samples, bit-identical excitation and measurement statistics across 27 configurations\n",n);
}
