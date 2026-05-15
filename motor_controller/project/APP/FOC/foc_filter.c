#include "foc_filter.h"

/*========================================================
 * Low-Pass Filter
 *========================================================*/
bool lpf1_init(lpf1_t *lpf, float fc, float fs)
{
    if (lpf == NULL || fc <= 0.0f || fs <= 0.0f)
    {
        return false;
    }
    lpf->fc = fc;
    lpf->fs = fs;
    lpf->alpha = 2.0f * M_PI * fc / fs;
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
        float output = lpf->prev;
        lpf->prev = input + lpf->alpha * (input - lpf->prev);
        return output;
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
}