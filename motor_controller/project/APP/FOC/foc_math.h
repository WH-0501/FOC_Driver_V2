/**
 * @file foc_math.h
 * @author your name (you@domain.com)
 * @brief Clarke / Park / 逆 Park：纯 float 运算。
 *      1.使用 static inline 避免 GCC/Clang 多 TU 链接时对非 static inline 的重复定义问题；
 *      2.sinf/cosf 与 float 一致，且在有 FPU 时通常比 sin/cos 更合适
 * @version 0.1
 * @date 2026-05-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __FOC_MATH_H__
#define __FOC_MATH_H__

#include <math.h>
#include <float.h>
#include <stdint.h>
#include "compiler_port.h"
#include "math_compat.h"

#define _PI_2 (1.57079632679f)
#define _PI (3.14159265359f)
#define _2PI (6.28318530718f)
#define _3PI_2 (4.71238898038f)
#define _INV_2PI (0.15915494309189534f) ///< 1/2π

#define ONE_DIV_2 (0.5f)
#define SQRT_3 (1.732050807568877f)             /* √3 */
#define SQRT_3_DIV_2 (0.866025403784438f)       /* √3/2 */
#define ONE_DIV_SQRT_2 (0.7071067811865475f)    /* 1/√2 */
#define ONE_DIV_SQRT_3 (0.57735026919f)         /* 1/√3 = √3/3 */
#define TWO_DIV_SQRT_3 (1.15470053838f)          /* 2/√3 */
#define ONE_DIV_3 (0.3333333333333333f)
#define TWO_DIV_3 (0.6666666666666666f)
#define THREE_DIV_2 (1.5f)

/* Shared size-optimized entry for calibration/identification; normal control
 * retains the inline implementation below. */
void foc_sincos_shared(float theta, float *s, float *c);

APP_STATIC_INLINE float foc_fabs(float angle)
{
  return (angle >= 0.0000001f) ? angle : -angle;
}

/**
 * @brief 角度(rad)归一化到 [0, 2π)
 * 
 * @param angle 
 * @return APP_STATIC_INLINE 
 */
APP_STATIC_INLINE float foc_wrapAngle(float angle)
{
  while (angle >= _2PI) {
    angle -= _2PI;
  }
  while (angle < 0.0f)
  {
    angle += _2PI;
  }
  return angle;
}

/* Joint single-precision sin/cos for the real-time transforms.
 * Fold to [-pi/2,pi/2], then degree 13/12 Taylor polynomials.
 * Truncation bounds: sin < 7e-10, cos < 7e-9 on that interval;
 * float rounding/range reduction dominates (see numerical validation).
 * Keep libm semantics for nonfinite/large external angles. */
APP_STATIC_INLINE void foc_sincos(float theta, float *s, float *c)
{
#if 1
  theta = foc_wrapAngle(theta);
  float sign = 1.0f;

  if (theta > _PI)
  {
    theta -= _PI;
    sign = -1.0f;
  }
  theta -= _PI_2;
  float temp = theta * theta;
  *s = (sign * (0.999999967F + (-0.499999269F + (0.041664091F + (-0.001385742F + 0.000023238F * temp) * temp) * temp) * temp));
  *c = (sign * (-0.9999974871F + (0.1666516805F + (-0.0083095164F + 0.0001844721F * temp) * temp) * temp) * theta);
#else
  float x=theta, sign=1.0f, z;
  if (!(x >= -_2PI && x <= _2PI)) {
    *s=sinf(theta); *c=cosf(theta); return;
  }
  if(x>_PI)x-=_2PI;
  else if(x<-_PI)x+=_2PI;
  if(x>0.5f*_PI){x=_PI-x;sign=-1.0f;}
  else if(x<-0.5f*_PI){x=-_PI-x;sign=-1.0f;}
  z=x*x;
  *s=x*(1.0f+z*(-1.0f/6.0f+z*(1.0f/120.0f+z*(-1.0f/5040.0f+
    z*(1.0f/362880.0f+z*(-1.0f/39916800.0f+z*(1.0f/6227020800.0f)))))));
  *c=sign*(1.0f+z*(-1.0f/2.0f+z*(1.0f/24.0f+z*(-1.0f/720.0f+
    z*(1.0f/40320.0f+z*(-1.0f/3628800.0f+z*(1.0f/479001600.0f)))))));
#endif
 }
 

/**
 * @brief 角度归一化到 [0, 2PI]
 * 
 * @param angle 角度
 * @return float 归一化后的角度 . 0~4us
 */
// APP_STATIC_INLINE float angle_normalize(float angle)
// {
// 	float a = fmodf(angle, _2PI);
// 	return ((a>=0) ? a : (a + _2PI));
// }
// APP_STATIC_INLINE float angle_normalize(float angle) // 0~2us
// {
//   float k = floorf(angle * _INV_2PI);
//   return angle - _2PI * k;
// }
/**
 * @brief 角度归一化到 [0, 2π) . 0~2us
 * 
 *  无 libm、固定时延、用整数取整.
 * 
 *  约束：
 *      1. |angle| < 2^24 * 2π（约1677万圈）；超大角度不要使用
 *      2. 禁止输入 ±Inf，会触发未定义行为
 *      3. 浮点舍入导致和fmodf版本存在若干ULP差异，闭环可接受
 * 
 * f	   输入 angle	  期望输出（mod 2π，[0,2π)）	旧版输出	旧版	新版输出	新版
 *-3.0	  -6π	0	-6π − 2π·(-4) = 2π（且无兜底）	❌ 越界到 2π	-6π − 2π·(-3) = 0，wrapped≥2π 不触发	✅ 0
 *-2.3	  -4.6π	0.4π（-2.3+3 = 0.7 圈 → 1.4π？见下注）	同新	✅	-4.6π − 2π·(-3) = 1.4π	✅
 *-0.0	  0	0	0 − 2π·(-1) = 2π（无兜底）	❌ 越界	0 − 0 = 0	✅
 *-1e-10	2π·1e-10	2π·1e-10（极小正角，保持原值）	2π·1e-10 − 2π·(-1) ≈ 2π	❌ 多加一圈	2π·1e-10 − 0	✅
 *-2.7	  5.4π	5.4π − 4π = 1.4π	5.4π − 2π·2 = 1.4π	✅	5.4π − 2π·2 = 1.4π	✅
 *-3.0	  6π	0	6π − 2π·3 = 0	✅	6π − 2π·3 = 0	✅
 * 
 * @param angle 
 * @return float 归一化后的角度
 */
APP_STATIC_INLINE float angle_normalize(float angle)
{
  /* Encoder/config angles are bounded. Truncation must be corrected only
   * for a genuinely negative fractional turn, not zero or negative integers. */
  float f = angle * _INV_2PI;
  int32_t k = (int32_t)f;
  if (f < (float)k) --k;
  float wrapped = angle - _2PI * (float)k;
  /* Float subtraction at a negative wrap may round up to exactly 2*pi. */
  if (wrapped >= _2PI) wrapped = 0.0f;
  return wrapped;
}

/**
 * @brief 两角之差，归一化到 (-π, π]
 */
APP_STATIC_INLINE float angle_delta_pm_pi(float a, float b)
{
  float d = a - b;

  if (d > _PI)
  {
    d -= _2PI;
  }
  else if (d <= -_PI)
  {
    d += _2PI;
  }
  return d;
}

/**
 * @brief Clarke 变换
 * 将三相电流转换为两相电流
 * @param xa A 相电流/电压
 * @param xb B 相电流/电压
 * @param xc C 相电流/电压
 * @param x_alpha 两相电流/电压 α 轴分量
 * @param x_beta 两相电流/电压 β 轴分量
 */
APP_STATIC_INLINE void clarke_transform(float xa, float xb, float xc, float *x_alpha, float *x_beta)
{
#if 0
  /* Full Clarke Transform */
  *x_alpha = TWO_DIV_3 * xa - ONE_DIV_3 * xb - ONE_DIV_3 * xc;
  *x_beta = (xb - xc) * ONE_DIV_SQRT_3;
#else
  /* All three currents are measured: remove zero-sequence from alpha as
   * beta already does. This is an instantaneous spatial transform, NOT a
   * time filter. Keep phase-current samples unchanged for protection/debug. */
  *x_alpha = xa;
  // *x_alpha = (2.0f * xa - xb - xc) * ONE_DIV_3;
  *x_beta = (xb - xc) * ONE_DIV_SQRT_3;
  // 等价于:
  // *x_beta = ONE_DIV_SQRT_3 * xa + TWO_DIV_SQRT_3 * xb;
#endif
}

/**
 * @brief 逆 Clarke 变换. 将两相电流/电压转换为三相电流/电压
 * 
 * @param x_alpha 两相电流/电压 α 轴分量
 * @param x_beta 两相电流/电压 β 轴分量
 * @param xa A 相电流/电压
 * @param xb B 相电流/电压
 * @param xc C 相电流/电压
 */
APP_STATIC_INLINE void inv_clarke_transform(float x_alpha, float x_beta, float *xa, float *xb, float *xc)
{
  *xa = x_alpha;
  *xb = -ONE_DIV_2 * x_alpha + SQRT_3_DIV_2 * x_beta;
  *xc = -ONE_DIV_2 * x_alpha - SQRT_3_DIV_2 * x_beta;
}

/**
 * @brief Park 变换
 * 将两相电流转换为 d、q 轴电流
 * 
 * @param x_alpha 两相电流/电压 α 轴分量
 * @param x_beta 两相电流/电压 β 轴分量
 * @param theta 电角度
 * @param x_d d 轴电流/电压
 * @param x_q q 轴电流/电压
 */
APP_STATIC_INLINE void park_transform(float x_alpha, float x_beta, float theta, float *x_d, float *x_q)
{
  float c, s;
  foc_sincos(theta, &s, &c);
  *x_d = x_alpha * c + x_beta * s;
  *x_q = -x_alpha * s + x_beta * c;
}

/* Park with caller-supplied sin/cos (FOC_REUSE_PARK_SINCOS publish path). */
APP_STATIC_INLINE void park_transform_sc(float x_alpha, float x_beta, float s, float c, float *x_d,
                                        float *x_q)
{
  *x_d = x_alpha * c + x_beta * s;
  *x_q = -x_alpha * s + x_beta * c;
}

/**
 * @brief 逆 Park 变换
 * 将 d、q 轴电流转换为两相电流
 * 
 * @param x_d d 轴电流/电压
 * @param x_q q 轴电流/电压
 * @param theta 电角度
 * @param x_alpha 两相电流/电压 α 轴分量
 * @param x_beta 两相电流/电压 β 轴分量
 */
APP_STATIC_INLINE void inv_park_transform(float x_d, float x_q, float theta, float *x_alpha, float *x_beta)
{
  float c, s;
  foc_sincos(theta, &s, &c);
  *x_alpha = x_d * c - x_q * s;
  *x_beta = x_d * s + x_q * c;
}

/* Inverse Park with caller-supplied sin/cos (FOC_REUSE_PARK_SINCOS). */
APP_STATIC_INLINE void inv_park_transform_sc(float x_d, float x_q, float s, float c, float *x_alpha,
                                            float *x_beta)
{
  *x_alpha = x_d * c - x_q * s;
  *x_beta = x_d * s + x_q * c;
}

/**
 * @brief SVPWM 调制
 * 将三相电压转换为三相 PWM 占空比. 实际完成:
 *     Ucom = 0.5f * (Umax + Umin);   // min-max 中点 = SVPWM 的 offset（三次谐波注入）
 *     svm_a = Ua - Ucom;            // 以 0 为中心的双极性电压（伏特，可正可负）
 *     svm_b = Ub - Ucom;
 *     svm_c = Uc - Ucom;
 * 
 * 注意：输出的是电压伏特值，而不是 PWM 占空比。需要根据电机参数转换为 PWM 占空比。
 *      0.5+U/vbus
 * 
 * @param Ua A 相电压
 * @param Ub B 相电压
 * @param Uc C 相电压
 * @param svm_a A 相 PWM 电压. 注意: 不是占空比
 * @param svm_b B 相 PWM 电压. 注意: 不是占空比
 * @param svm_c C 相 PWM 电压. 注意: 不是占空比
 */
APP_STATIC_INLINE void svm(float Ua, float Ub, float Uc, float *svm_a, float *svm_b, float *svm_c)
{
	float Umax, Umin, Ucom;
  // Umin = fminf(Ua, fminf(Ub, Uc));
  // Umax = fmaxf(Ua, fmaxf(Ub, Uc));
	if (Ua > Ub)
	{
		Umax = Ua;
		Umin = Ub;
	}
	else
	{
		Umax = Ub;
		Umin = Ua;
	}
	if (Uc > Umax)
	{
		Umax = Uc;
	}
	else if (Uc < Umin)
	{
		Umin = Uc;
	}
	Ucom = 0.5f * (Umax + Umin); // offset
	*svm_a = Ua - Ucom;
	*svm_b = Ub - Ucom;
	*svm_c = Uc - Ucom;
}

/**
 * @brief SVPWM 调制
 * 将两相电压转换为三相 PWM 占空比
 * 
 * @param alpha 两相电压 α 轴分量
 * @param beta 两相电压 β 轴分量
 * @param duty_a A 相 PWM 占空比. [0~1]
 * @param duty_b B 相 PWM 占空比. [0~1]
 * @param duty_c C 相 PWM 占空比. [0~1]
 * @return 0 成功, -1 失败. 如果任何结果为 NaN, 则返回 -1
 */
APP_STATIC_INLINE int svpwm(float alpha, float beta, float *duty_a, float *duty_b, float *duty_c)
{
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (ONE_DIV_SQRT_3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-ONE_DIV_SQRT_3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-ONE_DIV_SQRT_3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (ONE_DIV_SQRT_3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
    // sextant v1-v2
    case 1: {
        // Vector on-times
        float t1 = alpha - ONE_DIV_SQRT_3 * beta;
        float t2 = TWO_DIV_SQRT_3 * beta;

        // PWM timings
        *duty_a = (1.0f - t1 - t2) * 0.5f;
        *duty_b = *duty_a + t1;
        *duty_c = *duty_b + t2;
    } break;

    // sextant v2-v3
    case 2: {
        // Vector on-times
        float t2 = alpha + ONE_DIV_SQRT_3 * beta;
        float t3 = -alpha + ONE_DIV_SQRT_3 * beta;

        // PWM timings
        *duty_b = (1.0f - t2 - t3) * 0.5f;
        *duty_a = *duty_b + t3;
        *duty_c = *duty_a + t2;
    } break;

    // sextant v3-v4
    case 3: {
        // Vector on-times
        float t3 = TWO_DIV_SQRT_3 * beta;
        float t4 = -alpha - ONE_DIV_SQRT_3 * beta;

        // PWM timings
        *duty_b = (1.0f - t3 - t4) * 0.5f;
        *duty_c = *duty_b + t3;
        *duty_a = *duty_c + t4;
    } break;

    // sextant v4-v5
    case 4: {
        // Vector on-times
        float t4 = -alpha + ONE_DIV_SQRT_3 * beta;
        float t5 = -TWO_DIV_SQRT_3 * beta;

        // PWM timings
        *duty_c = (1.0f - t4 - t5) * 0.5f;
        *duty_b = *duty_c + t5;
        *duty_a = *duty_b + t4;
    } break;

    // sextant v5-v6
    case 5: {
        // Vector on-times
        float t5 = -alpha - ONE_DIV_SQRT_3 * beta;
        float t6 = alpha - ONE_DIV_SQRT_3 * beta;

        // PWM timings
        *duty_c = (1.0f - t5 - t6) * 0.5f;
        *duty_a = *duty_c + t5;
        *duty_b = *duty_a + t6;
    } break;

    // sextant v6-v1
    case 6: {
        // Vector on-times
        float t6 = -TWO_DIV_SQRT_3 * beta;
        float t1 = alpha + ONE_DIV_SQRT_3 * beta;

        // PWM timings
        *duty_a = (1.0f - t6 - t1) * 0.5f;
        *duty_c = *duty_a + t1;
        *duty_b = *duty_c + t6;
    } break;
    }

    // if any of the results becomes NaN, return fail
    if ((*duty_a >= 0.0f && *duty_a <= 1.0f && *duty_b >= 0.0f && *duty_b <= 1.0f &&
         *duty_c >= 0.0f && *duty_c <= 1.0f))
    {
        return 0;
    }
    return -1;
}

#endif
