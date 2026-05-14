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

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/// TODO: sin/cos 查表或 CORDIC 加速（与平台无关，按帧率取舍）
#define SQRT_3 (1.732050807568877f)             /* √3 */
#define SQRT_3_DIV_2 (0.866025403784438f)       /* √3/2 */
#define ONE_DIV_SQRT_2 (0.7071067811865475f)    /* 1/√2 */
#define ONE_DIV_SQRT_3 (0.57735026919f)         /* 1/√3，功率不变 Clarke 常用 */
#define TWO_BY_SQRT_3 (1.15470053838f)          /* 2/√3 */

/**
 * @brief Clarke 变换
 * 将三相电流转换为两相电流
 * @param ia 相电流 A 相电流
 * @param ib 相电流 B 相电流
 * @param ic 相电流 C 相电流
 * @param i_alpha 两相电流 α 轴分量
 * @param i_beta 两相电流 β 轴分量
 */
static inline void clarke_transform(float ia, float ib, float ic, float *i_alpha, float *i_beta)
{
  *i_alpha = ia;
  *i_beta = (ib - ic) * ONE_DIV_SQRT_3;
}

/**
 * @brief 逆 Clarke 变换. 将两相电流转换为三相电流
 * 
 * @param i_alpha 两相电流 α 轴分量
 * @param i_beta 两相电流 β 轴分量
 * @param ia A 相电流
 * @param ib B 相电流
 * @param ic C 相电流
 */
static inline void inv_clarke_transform(float i_alpha, float i_beta, float *ia, float *ib, float *ic)
{
  *ia = i_alpha;
  *ib = i_beta * SQRT_3_DIV_2 + i_alpha * ONE_DIV_SQRT_3;
  *ic = i_beta * SQRT_3_DIV_2 - i_alpha * ONE_DIV_SQRT_3;
}

/**
 * @brief Park 变换
 * 将两相电流转换为 d、q 轴电流
 * 
 * @param i_alpha 两相电流 α 轴分量
 * @param i_beta 两相电流 β 轴分量
 * @param theta 电角度
 * @param i_d d 轴电流
 * @param i_q q 轴电流
 */
static inline void park_transform(float i_alpha, float i_beta, float theta, float *i_d, float *i_q)
{
  const float c = cosf(theta);
  const float s = sinf(theta);
  *i_d = i_alpha * c + i_beta * s;
  *i_q = -i_alpha * s + i_beta * c;
}

/**
 * @brief 逆 Park 变换
 * 将 d、q 轴电流转换为两相电流
 * 
 * @param d d 轴电流
 * @param q q 轴电流
 * @param theta 电角度
 * @param i_alpha 两相电流 α 轴分量
 * @param i_beta 两相电流 β 轴分量
 */
static inline void inv_park_transform(float d, float q, float theta, float *i_alpha, float *i_beta)
{
  const float c = cosf(theta);
  const float s = sinf(theta);
  *i_alpha = d * c - q * s;
  *i_beta = d * s + q * c;
}

/**
 * @brief 空间矢量脉宽调制
 * 将三相电压转换为三相 PWM 占空比
 * @param Ua A 相电压
 * @param Ub B 相电压
 * @param Uc C 相电压
 * @param svm_a A 相 PWM 占空比
 * @param svm_b B 相 PWM 占空比
 * @param svm_c C 相 PWM 占空比
 */
static inline void svpwm(float Ua, float Ub, float Uc, float *svm_a, float *svm_b, float *svm_c)
{
	float Umax, Umin, Ucom;
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
	Ucom = 0.5f * (Umax + Umin);
	*svm_a = Ua - Ucom;
	*svm_b = Ub - Ucom;
	*svm_c = Uc - Ucom;
}

#endif