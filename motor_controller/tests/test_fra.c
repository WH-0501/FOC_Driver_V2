#include "fra.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define PI 3.14159265358979323846
static void test(unsigned method){
  const double fs=20000,alpha=exp(-2*PI*800/fs);
  double y=0;unsigned n=0,i;fra_result_t r;
  assert(fra_start(method,16,20,2000,.2f,(float)fs,123)==0);
  assert(fra_start(method,16,20,2000,.2f,(float)fs,123)==1);
  while(fra_state()==1){
    float x=fra_step((float)y);
    assert(fabs(x)<=.2001);
    y=alpha*y+(1-alpha)*x;
    assert(++n<1000000);
  }
  assert(fra_state()==2 && fra_progress()==16);
  if(method==1)assert(n==4000);
  for(i=0;i<16;i++){
    double w,re,im,den,actual_db,expected_db;
    assert(fra_result(i,&r)==0 && r.blocks==(method==1?3:4));
    w=2*PI*r.hz/fs;den=1+alpha*alpha-2*alpha*cos(w);
    re=(1-alpha)*(cos(w)-alpha)/den;im=-(1-alpha)*sin(w)/den;
    actual_db=20*log10(hypot(r.real,r.imag));expected_db=20*log10(hypot(re,im));
    assert(fabs(actual_db-expected_db)<.15);
    assert(fabs(atan2(r.imag,r.real)-atan2(im,re))<.03);
    assert(r.coherence>.99);
  }
  fra_release();printf("PASS method %u: %u samples, coherent H1 vs 800 Hz discrete plant\n",method,n);
}
int main(void){
  test(1);test(2);
  {fra_result_t r;double y=0,alpha=exp(-2*PI*800/20000.);unsigned i;
   assert(fra_start(2,16,300,1200,.2f,20000,128)==0);
   while(fra_state()==1){float x=fra_step((float)y);y=alpha*y+(1-alpha)*x;}
   for(i=0;i<2;i++){assert(!fra_result(i,&r));assert(fabs(r.hz-(i==0?20:40))<.01);assert(fabs(20*log10(hypot(r.real,r.imag)))<.02);}
   assert(!fra_result(2,&r));assert(r.hz>=290 && r.hz<=300);
   fra_release();puts("PASS narrow-band refinement retains 20/40 Hz reference");
  }
  assert(fra_start(1,16,20,2000,.2f,20000,124)==0);
  fra_abort(8);assert(fra_state()==3 && fra_flags()==8);assert(fra_step(0)==0);fra_release();
  assert(fra_start(1,16,20,2000,.2f,20000,125)==0);
  fra_timing(46);assert(fra_state()==3 && fra_flags()==4 && fra_timing_reason()==2 && fra_max_us()==46);fra_release();
  assert(fra_start(1,16,20,40,.2f,20000,126)==2);
  assert(fra_start(2,16,20,2000,1,20000,126)==2);
  {unsigned seed=123,weak=0,i;fra_result_t r;
   assert(fra_start(1,16,20,2000,.005f,20000,127)==0);
   while(fra_state()==1){seed=1664525u*seed+1013904223u;fra_step(((seed>>8)/(float)0xffffff-.5f)*.2f);}
   for(i=0;i<16;i++){assert(!fra_result(i,&r));if(r.coherence<.9f)weak++;}
   assert(weak>8);fra_release();
  }
  assert(fra_start(1,16,20,2000,.05f,20000,130)==0);
  fra_step(0);fra_interval(50);fra_interval(75);
  assert(fra_state()==3 && fra_timing_reason()==1 && fra_failure_sample()==1 && fra_min_interval()==50 && fra_max_interval()==75);fra_release();
  puts("PASS cancellation, timing rejection, parameters, peak bound and incoherent noise rejection");return 0;
}
