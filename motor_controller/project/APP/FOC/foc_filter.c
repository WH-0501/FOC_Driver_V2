#include "foc_filter.h"

#ifndef TWO_PI_F
#define TWO_PI_F (6.28318530717958648f)
#endif

float lpf1_filer(float Tf, float dt, float x, float y_prev)
{
    float alpha = dt / (dt + Tf);
    float y = x * alpha + y_prev * (1.0f - alpha);
}

/*========================================================
 * Low-Pass Filter
 *========================================================*/
bool lpf1_init(lpf1_t *lpf, float fc, float fs)
{
    /* fc 须低于奈奎斯特频率，否则离散近似无意义 */
    if (lpf == NULL || fc <= 0.0f || fs <= 0.0f || fc >= 0.5f * fs)
    {
        return false;
    }
    float ts = 1.0f / fs;
    lpf->fc = fc;
    lpf->fs = fs;
    lpf->tau = 1.0f / (TWO_PI_F * fc);
    lpf->alpha = ts / (lpf->tau + ts); /* y = alpha*x + (1-alpha)*y_prev */
    lpf->prev = 0.0f;
    return true;
}

bool lpf1_reset(lpf1_t *lpf, float initial_value)
{
    if (lpf != NULL)
    {
        lpf->prev = initial_value;
        return true;
    }
    return false;
} 

float lpf1_update(lpf1_t *lpf, float input)
{
    if (lpf != NULL)
    {
        float y =
            lpf->alpha * input + (1.0f - lpf->alpha) * lpf->prev;
        lpf->prev = y;
        return y;
    }
    return 0.0f;
}

/*========================================================
 * Second-order Low-Pass Filter (Butterworth)
 *========================================================*/
bool lpf2_init(lpf2_t *lpf, float fc, float fs, float Q)
{
    if (lpf == NULL || fc <= 0.0f || fs <= 0.0f || Q <= 0.0f)
    {
        return false;
    }
    (void)lpf;
    (void)fc;
    (void)fs;
    (void)Q;
    return false;
}