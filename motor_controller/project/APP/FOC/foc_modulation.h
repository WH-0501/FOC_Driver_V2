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
#include <stdint.h>
#include <string.h>
#include "common.h"

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

/* Binary32: a single exponent test rejects both NaN and infinity. */
#if defined(__CC_ARM)
static __forceinline
#else
APP_STATIC_INLINE
#endif
int foc_modulation_finite(float value)
{
#if FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && \
    (defined(__CC_ARM) || defined(_MSC_VER) || (defined(__SIZEOF_FLOAT__) && __SIZEOF_FLOAT__ == 4))
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
#else
  return value >= -FLT_MAX && value <= FLT_MAX;
#endif
}

/* Reserve a common low-side zero-vector window, preserving line voltages.
 * The linear circular limit is duty_max * Vbus / sqrt(3).
 * A common duty offset of duty_max/2 relocates the zero vectors; independently
 * clipping the three phases would change their differential voltages.
 * Returns 0 on invalid input. The output object is only committed on success.
 */
APP_STATIC_INLINE int foc_modulate_voltage_sincos(float ud, float uq, float sin_el, float cos_el,
                                                  float vbus, float duty_max, foc_modulation_t *out)
{
  foc_modulation_t next;
  float limit;
  float svm_a, svm_b, svm_c;
  float center, inverse_bus;

  if (out == NULL || !foc_modulation_finite(ud) || !foc_modulation_finite(uq) ||
      !foc_modulation_finite(sin_el) || !foc_modulation_finite(cos_el) ||
      !foc_modulation_finite(vbus) || !(vbus > 0.0f) || !(duty_max > 0.0f && duty_max <= 1.0f))
  {
    return 0;
  }

  limit = vbus * (duty_max * ONE_DIV_SQRT_3);
  if (!(limit > 0.0f)) return 0;

  float squared = ud * ud + uq * uq;
  float limit_squared = limit * limit;
  next.voltage_scale = 1.0f;
  if (squared <= FLT_MAX && limit_squared >= FLT_MIN)
  {
    if (squared > limit_squared)
    {
      next.voltage_scale = limit / sqrtf(squared);
      ud *= next.voltage_scale;
      uq *= next.voltage_scale;
    }
  }
  else
  {
    /* Scaling first avoids overflow for huge commands and underflow for tiny limits. */
    float peak = FMAX(fabsf(ud), fabsf(uq));
    if (peak > 0.0f)
    {
      float nd = ud / peak, nq = uq / peak;
      float magnitude = limit / sqrtf(nd * nd + nq * nq);
      if (peak > magnitude)
      {
        next.voltage_scale = magnitude / peak;
        ud = nd * magnitude;
        uq = nq * magnitude;
      }
    }
  }

  inv_park_transform_sc(ud, uq, sin_el, cos_el, &next.u_alpha, &next.u_beta);
  inv_clarke_transform(next.u_alpha, next.u_beta, &next.u_a, &next.u_b, &next.u_c);
  svm(next.u_a, next.u_b, next.u_c, &svm_a, &svm_b, &svm_c);
  center = 0.5f * duty_max;
  inverse_bus = 1.0f / vbus;
  if (foc_modulation_finite(inverse_bus))
  {
    next.duty_a = CLAMP(center + svm_a * inverse_bus, 0.0f, duty_max);
    next.duty_b = CLAMP(center + svm_b * inverse_bus, 0.0f, duty_max);
    next.duty_c = CLAMP(center + svm_c * inverse_bus, 0.0f, duty_max);
  }
  else
  {
    next.duty_a = CLAMP(center + svm_a / vbus, 0.0f, duty_max);
    next.duty_b = CLAMP(center + svm_b / vbus, 0.0f, duty_max);
    next.duty_c = CLAMP(center + svm_c / vbus, 0.0f, duty_max);
  }
  if (!foc_modulation_finite(next.duty_a) || !foc_modulation_finite(next.duty_b) ||
      !foc_modulation_finite(next.duty_c))
  {
    return 0;
  }
  *out = next;
  return 1;
}

APP_STATIC_INLINE int foc_modulate_voltage(float ud, float uq, float angle,
                                          float vbus, float duty_max, foc_modulation_t *out)
{
  float sin_el, cos_el;
  if (!foc_modulation_finite(angle)) return 0;
  foc_sincos(angle, &sin_el, &cos_el);
  return foc_modulate_voltage_sincos(ud, uq, sin_el, cos_el, vbus, duty_max, out);
}

#endif
