/**
 * @file foc_modulation.h
 * @author your name (you@domain.com)
 * @brief 将 Ud/Uq 转换为三相占空比，并在板级 duty_max（例如 AT32 约 84.84%）约束下，
 *        保证 ADC 触发时刻三相下管仍有足够共同导通窗口。
 * @version 0.1
 * @date 2026-10-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef FOC_MODULATION_H
#define FOC_MODULATION_H

#include <float.h>
#include <stddef.h>
#include "foc_math.h"

/**
 * @brief 调制结构体
 * 
 * @param u_alpha: alpha轴电压
 * @param u_beta: beta轴电压
 * @param u_a: a相电压
 * @param u_b: b相电压
 * @param u_c: c相电压
 * @param duty_a: a相占空比
 * @param duty_b: b相占空比
 * @param duty_c: c相占空比
 * @param voltage_scale: 电压缩放因子
 */
typedef struct
{
  float u_alpha, u_beta;
  float u_a, u_b, u_c;
  float duty_a, duty_b, duty_c;
  float voltage_scale;
} foc_modulation_t;

/* C99 math libraries differ between ARMCC and GCC; ordered finite bounds
 * also reject NaN and infinities without relying on a platform isfinite macro.
 * Build the control code without unsafe/finite-only math assumptions. */
APP_STATIC_INLINE int foc_modulation_finite(float value)
{
  return value >= -FLT_MAX && value <= FLT_MAX;
}

/* Reserve a common low-side zero-vector window, preserving line voltages.
 * The linear circular limit is duty_max * Vbus / sqrt(3).
 * A common duty offset of duty_max/2 relocates the zero vectors; independently
 * clipping the three phases would change their differential voltages.
 * Returns 0 on invalid input. The output object is only committed on success.
 */
APP_STATIC_INLINE int foc_modulate_voltage(float ud, float uq, float angle,
                                          float vbus, float duty_max,
                                          foc_modulation_t *out)
{
  foc_modulation_t next;
  float peak, norm, limit;
  float svm_a, svm_b, svm_c;
  float center;

  if (out == NULL || !foc_modulation_finite(ud) || !foc_modulation_finite(uq) ||
      !foc_modulation_finite(angle) || !foc_modulation_finite(vbus) ||
      !(vbus > 0.0f) || !(duty_max > 0.0f && duty_max <= 1.0f))
  {
    return 0;
  }

  /* Normalize first to avoid squaring large input voltages. */
  peak = FMAX(fabsf(ud), fabsf(uq));
  /* 限幅圆从 vbus * sqrt(3) 改为 vbus * duty_max * sqrt(3)，与硬件最大 CCR 一致 */
  limit = vbus * (duty_max * ONE_DIV_SQRT_3);
  if (!(limit > 0.0f))
  {
    return 0;
  }
  next.voltage_scale = 1.0f;
  if (peak > 0.0f)
  {
    float nd = ud / peak;
    float nq = uq / peak;
    norm = sqrtf(nd * nd + nq * nq);
    if (peak > limit / norm)
    {
      float magnitude = limit / norm;
      next.voltage_scale = (limit / peak) / norm;
      ud = nd * magnitude;
      uq = nq * magnitude;
    }
  }

  inv_park_transform(ud, uq, angle, &next.u_alpha, &next.u_beta);
  inv_clarke_transform(next.u_alpha, next.u_beta, &next.u_a, &next.u_b, &next.u_c);
  svm(next.u_a, next.u_b, next.u_c, &svm_a, &svm_b, &svm_c);
  /* 零矢量中心从 0.5 改为 duty_max/2，保证 ADC 触发时刻三相下管仍有足够共同导通窗口 */
  center = 0.5f * duty_max;
  /* The vector limit guarantees the bounds analytically. CLAMP only absorbs
   * roundoff at the boundary, never acts as the modulation limiter. */
  next.duty_a = CLAMP(center + svm_a / vbus, 0.0f, duty_max);
  next.duty_b = CLAMP(center + svm_b / vbus, 0.0f, duty_max);
  next.duty_c = CLAMP(center + svm_c / vbus, 0.0f, duty_max);
  if (!foc_modulation_finite(next.duty_a) || !foc_modulation_finite(next.duty_b) ||
      !foc_modulation_finite(next.duty_c))
  {
    return 0;
  }
  *out = next;
  return 1;
}

#endif
