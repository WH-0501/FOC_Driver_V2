/**
 * @file foc_filter.h
 * @author your name (you@domain.com)
 * @brief This file provides C implementations of commonly used filters in FOC motor control:
 * - First-order Low-Pass Filter (LPF)
 * - Second-order Low-Pass Filter (Butterworth)
 * - Notch Filter (for suppressing PWM carrier and harmonic noise)
 * - Band-Pass Filter
 * - Moving Average Filter
 * - Median Filter
 * - State Variable Filter (SVF)
 * - PI-based Filter / Integrator with Anti-windup
 * 
 * @version 0.1
 * @date 2026-05-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __FOC_FILTER_H__
#define __FOC_FILTER_H__

#include <stdint.h>
#include <stdbool.h>
#include "foc_math.h"

#ifndef FOC_FLT_EPSILON
#define FOC_FLT_EPSILON     1e-6f
#endif

/* Max order supported for IIR filters */
#ifndef FOC_MAX_FILTER_ORDER
#define FOC_MAX_FILTER_ORDER    4
#endif

/* Max window size for moving average filter */
#ifndef FOC_MAX_MA_WINDOW
#define FOC_MAX_MA_WINDOW       32
#endif

/* Max window size for median filter */
#ifndef FOC_MAX_MEDIAN_WINDOW
#define FOC_MAX_MEDIAN_WINDOW   16
#endif

/**
 * @brief 一阶低通滤波器结构体
 * 
 * 用于平滑噪声信号
 * y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 * 
 * 用 当前采样值 和 上一次平滑值 加权平均，α 越小，越依赖上一次的平滑值，滤波效果越强
 */
typedef struct _lpf1_t {
    float alpha;    ///< Ts/(τ+Ts)，离散式 y[n]=α·x[n]+(1-α)·y[n-1]（Ts=1/fs）
    float prev;     ///< 上一时刻输出
    float fc;       ///< 截止频率. 允许通过的最高频率（高于此频率的信号被衰减）. 建议设置为 PWM 频率的 1/10
    float fs;       ///< 采样频率. FOC 控制周期（电流环 与 PWM 周期同步）
    float tau;      ///< 时间常数. 滤波器响应速度（τ = 1 / (2π * fc). τ 越大，滤波越强，滞后越明显
} lpf1_t;

typedef struct _lpf2_t {
    float b0, b1, b2;   ///< 分子系数
    float a1, a2;       ///< 分母系数
    float x0, x1;       ///< 输入样本
    float y0, y1;       ///< 输出样本
    float fc;           ///< 截止频率
    float fs;           ///< 采样频率
    float Q;            ///< 品质因数
} lpf2_t;

/**
 * @brief 陷波滤波器结构体
 * 
* 用于抑制特定频率(如 PWM 载波频率、开关噪声或特定机械共振)
 */
typedef struct _notch_filter_t {
    float b0, b1, b2;   ///< 分子系数
    float a1, a2;       ///< 分母系数
    float x0, x1;       ///< 输入样本
    float y0, y1;       ///< 输出样本
    float fn;           ///< 陷波频率
    float fs;           ///< 采样频率
    float bw;           ///< 带宽
} notch_t;

/**
 * @brief 带通滤波器结构体
 * 
 * 用于通过特定频率范围(如电机谐波、电源噪声)
 * Transfer function: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 */
typedef struct _band_pass_filter_t {
    float b0, b1, b2;   ///< 分子系数
    float a1, a2;       ///< 分母系数
    float x1, x2;       ///< 输入样本
    float y1, y2;       ///< 输出样本
    float fc;           ///< 归一化频率
    float fs;           ///< 采样频率
    float Q;            ///< 品质因数
} bpf_t;

/**
 * @brief 滑动平均滤波器结构体
 * 
 * 用于平滑噪声信号
 * y[n] = (x[n] + x[n-1] + ... + x[n-N+1]) / N
 */
typedef struct _moving_average_filter_t {
    float buffer[FOC_MAX_MA_WINDOW];  ///< 缓冲区
    float sum;                        ///< 运行和
    uint16_t index;                  ///< 当前索引
    uint16_t window_size;            ///< 窗口大小 N
    uint16_t count;                  ///< 当前样本计数
} ma_t;

/**
 * @brief 中值滤波器结构体
 * 
 * 非线性滤波器，有效用于脉冲噪声去除
 * y[n] = median(x[n], x[n-1], ..., x[n-N+1])
 */
typedef struct _median_filter_t {
    float buffer[FOC_MAX_MEDIAN_WINDOW];  ///< 缓冲区
    uint16_t index;                  ///< 当前索引
    uint16_t window_size;            ///< 窗口大小
} median_t;

/**
 * @brief 状态变量滤波器 (SVF) - 通用滤波器
 * 
 * 同时提供 LPF, BPF, 和 HPF 输出
 * Transfer function: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 * 适用于谐振控制器应用
 */
typedef struct {
    float ic1eq;      ///< 状态 1 (积分器输出)
    float ic2eq;      ///< 状态 2 (积分器输出)
    float g;          ///< 截止频率的正切值 / 2
    float k;          ///< 阻尼因子 (1/Q)
    float a1;         ///< 系数
    float a2;         ///< 系数
    float a3;         ///< 系数
    float fc;         ///< 截止频率 [Hz]
    float fs;         ///< 采样频率 [Hz]
    float Q;          ///< 品质因数
} svf_t;

/**
 * @brief PI 滤波器 / 积分器与抗积分饱和
 * 
 * 在 FOC 电流环和速度环中常用
 * Transfer function: H(z) = Kp + Ki/z
 */
typedef struct {
    float Kp;         ///< 比例增益
    float Ki;         ///< 积分增益
    float integral;   ///< 积分累加器
    float out_max;    ///< 输出上限
    float out_min;    ///< 输出下限
    float int_max;    ///< 积分器上限 (抗积分饱和)
    float int_min;    ///< 积分器下限 (抗积分饱和)
} pi_filter_t;

/**
 * @brief 高通滤波器结构体
 * 
 * 用于去除电流测量中的直流偏移
 * Transfer function: H(z) = (1 - z^-1) / (1 - alpha*z^-1)
 */
typedef struct _hpf1_t {
    float alpha;      ///< 滤波系数
    float y_prev;     ///< 上一时刻输出
    float x_prev;     ///< 上一时刻输入
    float fc;         ///< 截止频率 [Hz]
    float fs;         ///< 采样频率 [Hz]
} hpf1_t;

/**
 * @brief 超前滞后补偿器结构体
 * 
 * 用于相位补偿在 FOC 控制环路中
 * Transfer function: H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
 */
typedef struct _lead_lag_compensator_t {
    float b0, b1;     ///< 分子系数
    float a1;         ///< 分母系数
    float x1;         ///< 输入历史
    float y1;         ///< 输出历史
} llc_t;

/*========================================================
 * Low-Pass Filter
 *========================================================*/
/**
 * @brief 初始化一阶低通滤波器（一阶 RC 离散化）
 *
 * τ=1/(2π·fc)，α=Ts/(τ+Ts)，Ts=1/fs。要求 fc \< fs/2。
 *
 * @param lpf 滤波器指针
 * @param fc 截止频率 [Hz]
 * @param fs 采样频率 [Hz]
 */
bool lpf1_init(lpf1_t *lpf, float fc, float fs);
bool lpf1_reset(lpf1_t *lpf, float initial_value);
/**
 * @brief 的一步更新：y=α·x+(1−α)·y_prev，返回当前输出 y
 */
float lpf1_update(lpf1_t *lpf, float input);

/*========================================================
 * Second-order Low-Pass Filter (Butterworth)
 *========================================================*/
bool lpf2_init(lpf2_t *lpf, float fc, float fs, float Q);
bool lpf2_reset(lpf2_t *lpf, float initial_value);
float lpf2_update(lpf2_t *lpf, float input);

bool notch_init(notch_t *notch, float fn, float fs, float bw);
bool notch_reset(notch_t *notch, float fn, float fs, float bw);
float notch_update(notch_t *notch, float input);

bool bpf_init(bpf_t *bpf, float fc, float fs, float Q);
bool bpf_reset(bpf_t *bpf, float fc, float fs, float Q);
float bpf_update(bpf_t *bpf, float input);

/*========================================================
 * Moving Average Filter
 *========================================================*/
bool ma_init(ma_t *ma, uint16_t window_size);
bool ma_reset(ma_t *ma, float initial_value);
float ma_update(ma_t *ma, float input);
bool ma_set_window_size(ma_t *ma, uint16_t window_size);

/*========================================================
 * Median Filter
 *========================================================*/

bool median_init(median_t *median, uint16_t window_size);
bool median_reset(median_t *median, float initial_value);
float median_update(median_t *median, float input);
bool median_set_window_size(median_t *median, uint16_t window_size);

/*========================================================
 * State Variable Filter (SVF)
 *========================================================*/

bool svf_init(svf_t *svf, float fc, float fs, float Q);
bool svf_reset(svf_t *svf, float initial_value);
float svf_update(svf_t *svf, float input);
bool svf_set_fc(svf_t *svf, float fc);
bool svf_set_fs(svf_t *svf, float fs);
bool svf_set_Q(svf_t *svf, float Q);

/*========================================================
 * PI Filter / Integrator with Anti-windup
 *========================================================*/

bool pi_filter_init(pi_filter_t *pi_filter, float Kp, float Ki, float out_max, float out_min, float int_max, float int_min);
bool pi_filter_reset(pi_filter_t *pi_filter, float initial_value);
float pi_filter_update(pi_filter_t *pi_filter, float input);
bool pi_filter_set_Kp(pi_filter_t *pi_filter, float Kp);
bool pi_filter_set_Ki(pi_filter_t *pi_filter, float Ki);
bool pi_filter_set_out_max(pi_filter_t *pi_filter, float out_max);
bool pi_filter_set_out_min(pi_filter_t *pi_filter, float out_min); 
bool pi_filter_set_int_max(pi_filter_t *pi_filter, float int_max);

#endif /* __FOC_FILTER_H__ */