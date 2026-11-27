#include "foc_math.h"
#if defined(__CC_ARM)
#pragma Ospace
#endif
void foc_sincos_shared(float theta, float *s, float *c)
{
  foc_sincos(theta, s, c);
}
