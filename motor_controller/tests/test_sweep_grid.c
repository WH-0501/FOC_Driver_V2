/* Test the actual private foreground helper, with libm as independent oracle. */
#include "../project/APP/FOC/sweep/sweep_engine.c"
#include <assert.h>
#include <stdio.h>
int main(void) {
  unsigned i,n,k,cases=0; double worst=0;
  for(i=0;i<300;++i) {
    float low=20.0f+(float)((i*71u)%1700u);
    float high=low+(2000.0f-low)*(float)(1u+i%99u)/100.0f;
    for(n=1;n<=15;++n) {
      double step=geometric_step(low,high,n), generated=low;
      for(k=0;k<=n;++k) {
        double reference=low*pow((double)high/low,(double)k/n);
        double error=fabs(generated/reference-1.0);
        if(error>worst)worst=error;
        assert(error<2e-12);
        generated*=step;++cases;
      }
    }
  }
  /* Worst supported ratio, including low-frequency anchors. */
  for(n=1;n<=15;++n) {
    double actual=geometric_step(20,2000,n);
    assert(fabs(actual/pow(100.0,1.0/n)-1)<2e-12);
  }
  printf("PASS geometric grid: %u points against double pow, max relative error %.3g; foreground only\n",cases,worst);
  return 0;
}
