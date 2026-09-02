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
#include <stdint.h>
#include "compiler_port.h"
#include "math_compat.h"

#define _PI (3.14159265359f)
#define _2PI (6.28318530718f)
#define _3PI_2 (4.71238898038f)

/// TODO: sin/cos 查表或 CORDIC 加速（与平台无关，按帧率取舍）
#define ONE_DIV_2 (0.5f)
#define SQRT_3 (1.732050807568877f)             /* √3 */
#define SQRT_3_DIV_2 (0.866025403784438f)       /* √3/2 */
#define ONE_DIV_SQRT_2 (0.7071067811865475f)    /* 1/√2 */
#define ONE_DIV_SQRT_3 (0.57735026919f)         /* 1/√3，功率不变 Clarke 常用. = √3/3 */
#define TWO_DIV_SQRT_3 (1.15470053838f)          /* 2/√3 */
#define ONE_DIV_3 (0.3333333333333333f)
#define TWO_DIV_3 (0.6666666666666666f)
#define THREE_DIV_2 (1.5f)
/**
 * @brief 角度归一化到 [0, 2PI]
 * 
 * @param angle 角度
 * @return float 归一化后的角度
 */
APP_STATIC_INLINE float angle_normalize(float angle)
{
	float a = fmodf(angle, _2PI);
	return ((a>=0) ? a : (a + _2PI));
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
  /* Assuming balanced three-phase system (xa + xb + xc = 0) */
  *x_alpha = xa;
  *x_beta = (xb - xc) * ONE_DIV_SQRT_3;
  // 等价于：
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
  const float c = cosf(theta);
  const float s = sinf(theta);
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
  const float c = cosf(theta);
  const float s = sinf(theta);
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
