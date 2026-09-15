#include "foc_filter.h"
#include "common.h"
#include <math.h>
#include <stddef.h>
#include <stdbool.h>

#ifndef TWO_PI_F
#define TWO_PI_F (6.28318530717958648f)
#endif

float lpf1_filer(float Tf, float dt, float x, float y_prev)
{
    float alpha = dt / (dt + Tf);
    float y = x * alpha + y_prev * (1.0f - alpha);
    return 0.0;
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

/**
 * @brief 更新低通滤波器输出
 *        1. 计算滤波器输出
 *        2. 更新滤波器输入
 * 注意: 要满足 fc < fs/2 的条件（奈奎斯特准则）, 否则离散近似无意义
 * @param lpf 低通滤波器句柄
 * @param input 输入信号
 * @return float 输出信号
 */
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

#if FOC_ENCODER_USE_PLL
/*========================================================
 * Encoder PLL (ODrive / DGM discrete phase detector)
 *========================================================*/
bool encoder_pll_init(encoder_pll_t *pll, float bandwidth_rad_s, float sample_period)
{
  if ((pll == NULL) || (bandwidth_rad_s <= 0.0f) || (sample_period <= 0.0f))
  {
    return false;
  }

  pll->kp = 2.0f * bandwidth_rad_s;
  pll->ki = 0.25f * pll->kp * pll->kp;
  pll->snap_threshold = 0.5f * sample_period * pll->ki;
  pll->innovation_gate_counts = FOC_ENCODER_PLL_INNOVATION_GATE_MAX_COUNTS;
  pll->pos_counts = 0.0f;
  pll->vel_counts = 0.0f;
  pll->last_delta_pos = 0.0f;
  pll->innovation_rejected = 0u;
  return (sample_period * pll->kp < 1.0f);
}

float encoder_pll_effective_gate(const encoder_pll_t *pll, float sample_period)
{
  float gate;
  float motion;

  if ((pll == NULL) || (pll->innovation_gate_counts <= 0.0f) ||
      (sample_period <= 0.0f))
  {
    return 0.0f;
  }

  motion = fabsf(pll->vel_counts) * sample_period;
  gate = motion * FOC_ENCODER_PLL_INNOVATION_GATE_VEL_FACTOR +
         FOC_ENCODER_PLL_INNOVATION_GATE_MIN_COUNTS;
  if (gate > pll->innovation_gate_counts)
  {
    gate = pll->innovation_gate_counts;
  }
  return gate;
}

void encoder_pll_reset(encoder_pll_t *pll, float count_in_cpr)
{
  if (pll == NULL)
  {
    return;
  }

  pll->pos_counts = count_in_cpr;
  pll->vel_counts = 0.0f;
  pll->last_delta_pos = 0.0f;
  pll->innovation_rejected = 0u;
}

void encoder_pll_update(encoder_pll_t *pll,
                        float count_in_cpr,
                        float sample_period,
                        float cpr,
                        float cpr_half)
{
  float delta_pos;

  if (pll == NULL)
  {
    return;
  }

  /* 1) 速度预测 */
  pll->pos_counts += sample_period * pll->vel_counts;

  /* 2) 离散鉴相：测量 CPR 内位置 − floor(估计位置) */
  delta_pos = count_in_cpr - floorf(pll->pos_counts);
  while (delta_pos > cpr_half)
  {
    delta_pos -= cpr;
  }
  while (delta_pos < -cpr_half)
  {
    delta_pos += cpr;
  }

  pll->last_delta_pos = delta_pos;

  /* innovation gate：野值帧仅保留预测，不拉偏 pos/vel */
  {
    float gate = encoder_pll_effective_gate(pll, sample_period);
    if ((gate > 0.0f) && (fabsf(delta_pos) > gate))
    {
      pll->innovation_rejected = 1u;
      while (pll->pos_counts >= cpr)
      {
        pll->pos_counts -= cpr;
      }
      while (pll->pos_counts < 0.0f)
      {
        pll->pos_counts += cpr;
      }
      return;
    }
  }
  pll->innovation_rejected = 0u;

  /* 3) PLL 反馈（ODrive / DGM 同款） */
  pll->pos_counts += sample_period * pll->kp * delta_pos;
  while (pll->pos_counts >= cpr)
  {
    pll->pos_counts -= cpr;
  }
  while (pll->pos_counts < 0.0f)
  {
    pll->pos_counts += cpr;
  }

  pll->vel_counts += sample_period * pll->ki * delta_pos;

  /* 4) 零速 snap，抑制静止抖动 */
  if (fabsf(pll->vel_counts) < pll->snap_threshold)
  {
    pll->vel_counts = 0.0f;
  }
}
#endif /* FOC_ENCODER_USE_PLL */
