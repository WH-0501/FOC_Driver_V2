#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include "foc_math.h"
#include "math_compat.h"
#include "dwt_profile_delay.h"

#define BUS_VOLTAGE     (24.0f) ///< 母线电压
#define POLE_PAIRS      (5) ///< 极对数
#define GEAR_RATIO      (50) ///< 电机到关节减速比（直驱填 1.0）

#define DEFAULT_TEMP    (25.0f) ///< 默认温度 [℃]
#define DEFAULT_TEMP_ADC (1588) ///< 默认温度 ADC 值

/* 滤波参数 0x88(默认),有效分辨率 13bit */
#define ENCODER_COUNTS_PER_REV (8192) ///< 机械旋转一圈编码器计数（分辨率定义依传感器）
#define ENCODER_COUNTS_PER_REV_MASK (0x1FFF) ///< 13bit 有效分辨率掩码
#define THETA_ELEC_OFFSET_RAD (0.0f) ///< 电角度零偏置 [rad]，对齐/校准后写入
#define THETA_OFFSET_RAD (0.0f) ///< 机械角度零偏置 [rad]，对齐/校准后写入

#define ENCODER_COUNTS_HALF_REV (ENCODER_COUNTS_PER_REV / 2) ///< 半圈计数，即 counts * 0.5
#define ENCODER_COUNTS_TURN_THRESH ((ENCODER_COUNTS_PER_REV * 4) / 5) ///< 0.8 圈计数，截断与 (int32_t)(0.8f * counts) 相同

/* 速度单位换算（内部统一使用 rad/s） */
#define RPM_TO_RAD_PER_SEC(rpm) ((float)(rpm) * (_2PI / 60.0f))
#define RAD_PER_SEC_TO_RPM(rad_s) ((float)(rad_s) * (60.0f / _2PI))

/* 限制参数 */
#define MAX_VELOCITY_RPM   (3000.0f)                         ///< 电机轴绝对最高速度 [RPM]（非减速输出轴；GUI不可修改）
#define MAX_VELOCITY_RAD_S (RPM_TO_RAD_PER_SEC(MAX_VELOCITY_RPM)) ///< 最大速度 [rad/s]
#define MAX_ACC_RAD_S2 (100.0f) ///< 最大加速度 [rad/s²]
#define MAX_JERK_RAD_S3 (1000.0f) ///< 最大加加速度 [rad/s³]

#define CPU_CLOCK_HZ (180000000.0f) ///< 180MHz


/* 角度原始值 -> 弧度，满量程对齐 2π（与 KTH71 定义为 uint16 满刻度时一致） */
#define RAW_ANGLE_U16_TO_RAD            (_2PI * (1.0f / ENCODER_COUNTS_PER_REV)) // 13bit 有效分辨率
#define DIFF_ANGLE_WRAP_THRESH_RAD      (0.8f * _2PI) // 超过此值视作跨一圈折返用于圈数校正
#define RAW_ANGLE_TO_RAD (1.5259e-5) // 除以 65536.0f


/* 电角度对齐时的电压参数 */
#define ALIGN_ELECTRICAL_ANGLE_RAD     (_3PI_2)
#define ALIGN_ELECTRICAL_VOLTAGE_D_V   (3.5f)
/** Uq 锁定拖拽电压；Ud 模式在代码中固定 Uq=0 */
#define ALIGN_ELECTRICAL_VOLTAGE_Q_V   (1.5f)
#define ALIGN_HOLD_MS                  (700u)
/** dex-con Uq 对齐稳定等待时间 [ms]：SetPhaseVoltage(1,0,_3PI_2) 后等待 */
#define ALIGN_HOLD_MS_DEX              (300u)
#define ALIGN_SAMPLE_COUNT             (16u)
#define ALIGN_SAMPLE_INTERVAL_MS       (5u)
#define ALIGN_RELOCK_MS                (200u)
/** 对齐后 +Vq 探针：i_q 为负则翻转 config.param.direction 并重算 offset */
#define ALIGN_POLARITY_TEST_UQ_V       (1.5f)
#define ALIGN_POLARITY_TEST_MS         (120u)
#define ALIGN_POLARITY_IQ_MIN_A        (0.03f)

/* 转子吸合验证：反相点二次锁定位移容差（rad，默认≈20° 电角度） */
#ifndef ALIGN_ROTOR_ENGAGE_TOL_RAD
#define ALIGN_ROTOR_ENGAGE_TOL_RAD (0.35f)
#endif

/* 锁定采样分散度上限（rad，默认≈5° 电角度）：超过视为未吸合到位/振荡 */
#ifndef ALIGN_ROTOR_SPREAD_MAX_RAD
#define ALIGN_ROTOR_SPREAD_MAX_RAD (0.09f)
#endif

#define OPEN_LOOP_UQ_RAMP_RATE      20.0f
#define OPEN_LOOP_OMEGA_RAMP_RATE   20.0f
#define OPEN_LOOP_UQ_MAX            1.0f // 2.0f
#define OPEN_LOOP_IQ_MAX            0.3f
/** 速度低于此阈值时不输出 Uq，避免零速堵转发热 [rad/s] */
#define OPEN_LOOP_OMEGA_EPS_RAD_S   (0.05f)

#if defined(PLATFORM_AT32)
#include "at32_foc_timing.h"
#define FOC_TMR1_CLK_HZ ((float)AT32_FOC_TIMER_CLOCK_HZ)
#define FOC_TMR1_DIV_REG AT32_FOC_TIMER_DIV_REG
#define FOC_TMR1_PERIOD AT32_FOC_TIMER_ARR
#else
#define FOC_TMR1_CLK_HZ (180000000.0f)
#define FOC_TMR1_DIV_REG (1u)
#define FOC_TMR1_PERIOD  (2249u)
#endif
#define FOC_PWM_HZ (FOC_TMR1_CLK_HZ / (2u *  (FOC_TMR1_DIV_REG + 1u) * (FOC_TMR1_PERIOD + 1u)))
#define FOC_PWM_PERIOD_TS    (1.0f / (float)FOC_PWM_HZ)

/*
 * FOC 电流环 / AT32 PWM 热点优化（A/B 对比：调试时只改本节 0/1，然后全量重编）。
 * board_at32.c 须在 #include "at32_foc_timing.h" 之前包含本头，AT32_PWM_* 才会生效。
 * 仅包含 at32_foc_timing.h 的单测仍使用该头内的 #ifndef 默认值。
 */
#ifndef FOC_JOINT_ENCODER_DIRECTION
#define FOC_JOINT_ENCODER_DIRECTION 0 /* 0 inherits motor direction, otherwise +1 or -1. */
#endif
#if FOC_JOINT_ENCODER_DIRECTION < -1 || FOC_JOINT_ENCODER_DIRECTION > 1
#error Invalid FOC_JOINT_ENCODER_DIRECTION
#endif
#ifndef FOC_JOINT_ENCODER_ZERO_RAD
#define FOC_JOINT_ENCODER_ZERO_RAD 0.0f
#endif
/* Control feedback and both outer PIDs run together on the first and every Nth tick. */
#ifndef FW_FOC_OUTER_LOOP_DIV
#define FW_FOC_OUTER_LOOP_DIV 5u
#endif
#if FW_FOC_OUTER_LOOP_DIV < 1 || FW_FOC_OUTER_LOOP_DIV > 255
#error FW_FOC_OUTER_LOOP_DIV must be between 1 and 255
#endif
/* RUNNING CST telemetry: 0 = position field: outer_tick (0/1), velocity field:
 * loop minus sense and control (us). 1 = encoder position/SPI/velocity breakdown (us).
 * 2 = joint position conversion and velocity estimate (raw cycles on outer ticks). */
#ifndef FOC_PROFILE_CST_OUTER
#define FOC_PROFILE_CST_OUTER 0
#endif
/* Park 正余弦算法：0 = 原 foc_sincos 多项式，1 = 512 点查表加角度补偿。 */
#ifndef FOC_PARK_USE_SINCOS_LUT
#define FOC_PARK_USE_SINCOS_LUT 0
#endif
#ifndef FOC_REUSE_PARK_SINCOS
#define FOC_REUSE_PARK_SINCOS 1 /* Park 一次 sincos，逆 Park 复用 sin_elec/cos_elec */
#endif
#ifndef AT32_PWM_INLINE_STAGE_CCR
#define AT32_PWM_INLINE_STAGE_CCR 1 /* pwm_stage_compare 内联写 c1/2/3dt */
#endif

/* 电流环 ISR：1=Id 纯 P + Iq 精简 PID（调制仍完整校验）；0=A/B */
#ifndef FOC_CUR_LOOP_FAST
#define FOC_CUR_LOOP_FAST 0
#endif

/** 1=ODrive/DGM PLL；0=原始方式（raw 角 + Δangle/dt + 速度 LPF，默认） */
#ifndef FOC_ENCODER_USE_PLL
#define FOC_ENCODER_USE_PLL (0)
#endif

#if FOC_ENCODER_USE_PLL
/** 编码器 PLL 带宽 [Hz]（ODrive/DGM：bandwidth_rad_s = 2π·Hz，Kp=2·bandwidth） */
#ifndef FOC_ENCODER_PLL_BW_HZ
#define FOC_ENCODER_PLL_BW_HZ (100.0f)
#endif
/** PLL innovation gate 上限 [counts]；0=关闭 gate */
#ifndef FOC_ENCODER_PLL_INNOVATION_GATE_MAX_COUNTS
#define FOC_ENCODER_PLL_INNOVATION_GATE_MAX_COUNTS (256.0f)
#endif
/** 静止/低速最小 gate [counts]，避免把正常步进当野值 */
#ifndef FOC_ENCODER_PLL_INNOVATION_GATE_MIN_COUNTS
#define FOC_ENCODER_PLL_INNOVATION_GATE_MIN_COUNTS (24.0f)
#endif
/** 有效 gate = vel·dt·factor + min，再截断到 max */
#ifndef FOC_ENCODER_PLL_INNOVATION_GATE_VEL_FACTOR
#define FOC_ENCODER_PLL_INNOVATION_GATE_VEL_FACTOR (4.0f)
#endif
#endif /* FOC_ENCODER_USE_PLL */

/* 机械 100RPM → 机械角速度 [rad/s] */
#define OL_OMEGA_100RPM_MECH   RPM_TO_RAD_PER_SEC(100.0f)

/** 12-bit ADC 标度（电流环、板级慢路径共用；可在工程宏中覆写） */
#ifndef FOC_ADC_VREF_V
#define FOC_ADC_VREF_V (3.3f)
#endif
#ifndef FOC_ADC_FULL_SCALE
#define FOC_ADC_FULL_SCALE (4095.0f)
#endif
#ifndef FOC_ADC_VOLTS_PER_LSB
#define FOC_ADC_VOLTS_PER_LSB (FOC_ADC_VREF_V / FOC_ADC_FULL_SCALE)
#endif

/**
 * 温度 ADC 线性化斜率 [V/℃]（与板载/电机温感同型公式；`board_get_vbus_and_temp` 更新 motor_temp）。
 * 单点标定来自 `motor_config_t::temp_adc` / `temp`。
 */
#ifndef FOC_TEMP_ADC_SLOPE_V_PER_C
#define FOC_TEMP_ADC_SLOPE_V_PER_C (-0.0042f)
#endif
#define FOC_TEMP_ADC_TO_CELSIUS(adc_raw, cal_adc, cal_temp_c)                       \
  (((float)((uint16_t)(cal_adc) - (uint16_t)(adc_raw)) * FOC_ADC_VOLTS_PER_LSB) /   \
   FOC_TEMP_ADC_SLOPE_V_PER_C + (float)(cal_temp_c))

#define DEF_UVLO_VOLTAGE (9.0f)
#define DEF_OVP_VOLTAGE (28.0f)
#define DEF_OCP_CURRENT (1.5f)
#define DEF_TEMP_LIMIT (85.0f)
#define DEF_SW_OCP (1.5f)
#define DEF_SW_OCP_HOLD_MS (1000.0f)
#define DEF_PHASE_DIAG_I_AVG_MIN_A (0.0f)
#define DEF_PHASE_IMBALANCE_RATIO_MAX (0.0f)
#define DEF_PHASE_IMBALANCE_WARN_RATIO (0.0f)

/* PID 参数 */
#define DEF_POS_KP (2.0f)
#define DEF_POS_KI (0.0f)
#define DEF_POS_KD (0.0f)
#define DEF_POS_RAMP (0.0f)
#define DEF_POS_LIMIT (0.0f)
#define DEF_POS_D_FILTER (0.0f)
#define DEF_VEL_KP (0.0008f)
#define DEF_VEL_KI (0.003f)
#define DEF_VEL_KD (0.0f)
#define DEF_VEL_RAMP (0.0f)
#define DEF_VEL_LIMIT (0.0f)
#define DEF_VEL_D_FILTER (0.0f)
// #define DEF_CUR_KP (1.8f)
// #define DEF_CUR_KI (0.22f)
// #define DEF_CUR_KD (0.0f)
// #define DEF_CUR_RAMP (0.0f)
// #define DEF_CUR_LIMIT (0.0f)
// #define DEF_CUR_D_FILTER (0.0f)
#define DEF_Q_KP (1.0f)
#define DEF_Q_KI (1507.96f)
#define DEF_Q_KD (0.0f)
#define DEF_Q_RAMP (0.0f)
#define DEF_Q_LIMIT (0.0f)
#define DEF_Q_D_FILTER (0.0f)
#define DEF_D_KP (1.0f)
#define DEF_D_KI (0.0f) // 1507.96f
#define DEF_D_KD (0.0f) 
#define DEF_D_RAMP (0.0f)
#define DEF_D_LIMIT (0.0f)
#define DEF_D_D_FILTER (0.0f)
#define DEF_D_ANTI_WINDUP_GAIN (0.5f)
#define DEF_Q_ANTI_WINDUP_GAIN (0.5f)
#define DEF_I_LIMIT_FACTOR (0.85f)

#define PID_DT_THRESHOLD (0.5f)
#define PID_DT_MAX (0.001f)

/* 是否使用速度环固定周期 */
#define USE_VEL_PERIOD        (1)

/* ==========================================================================
 * 一阶低通与测量链路（统一定义；改带宽/截止频率只改本节）
 * ========================================================================== */

/** 环路带宽设计参考 [Hz]（电流环 PID 尚未按该宏自动整定） */
#define FOC_CURRENT_BANDWIDTH_HZ (800.0f)
#define FOC_VEL_BANDWIDTH_HZ     (100.0f)
#define FOC_POS_BANDWIDTH_TAU    (40.0f)

/** 电流环：采样率与测量链 lpf1（Id/Iq/相电压/相电流等，见 foc_init_filters） */
#define FOC_CURRENT_LOOP_FS_HZ  (FOC_PWM_HZ)
#define FOC_CURRENT_MEAS_PERIOD ((float)(1.0f / (float)FOC_CURRENT_LOOP_FS_HZ))
#define FOC_MEAS_LPF_FS_HZ      (FOC_CURRENT_LOOP_FS_HZ)
#define FOC_MEAS_LPF_FC_HZ      (FOC_CURRENT_BANDWIDTH_HZ * 3.0f) /* 与速度链 fc=带宽×3 一致；须 < FOC_MEAS_LPF_FS_HZ/2 */
#define FOC_CURRENT_PERIOD_S    (1.0f / FOC_CURRENT_LOOP_FS_HZ)

/**
 * 速度观测（非 PLL）：差分 raw + 一阶 LPF
 * fc 取速度带宽的 3 倍；τ_us = 1/(2π·fc)，α = dt/(dt+τ)
 */
#define FOC_VEL_LPF_FC_HZ       (FOC_VEL_BANDWIDTH_HZ * 3.0f)
#define FOC_VEL_LPF_TAU         (1.0e6f / (_2PI * FOC_VEL_LPF_FC_HZ)) /* [µs] */
#define FOC_VEL_MEAS_PERIOD_S   ((float)(FOC_CURRENT_MEAS_PERIOD * (float)FW_FOC_OUTER_LOOP_DIV)) /* [s] */
#define FOC_VEL_MEAS_PERIOD_US  (FOC_VEL_MEAS_PERIOD_S * 1.0e6f) /* [µs]，与 FOC_VEL_LPF_TAU 同单位 */
/** v = Δθ/dt；用 1/dt [1/s] 替代除法（dt 取外环固定周期） */
#define FOC_INV_VEL_MEAS_PERIOD (1.0f / FOC_VEL_MEAS_PERIOD_S) /* [1/s] */
/** 1：编码器速度 LPF 用固定 FOC_VEL_LPF_ALPHA；0：用实测 dt_us 算 α */
#define FOC_VEL_LPF_ALPHA       (FOC_VEL_MEAS_PERIOD_US / (FOC_VEL_MEAS_PERIOD_US + FOC_VEL_LPF_TAU))

/**
 * 母线慢路径（foc_sense_poll_vbus，与电流环解耦）
 * fc 须 < FOC_VBUS_SAMPLE_FS_HZ/2，否则 lpf1_init 失败
 */
#define FOC_VBUS_SAMPLE_FS_HZ (100.0f)
#define FOC_VBUS_LPF_FC_HZ    (10.0f)
#define FOC_VBUS_POLL_MIN_INTERVAL_US ((uint32_t)(1000000.0f / FOC_VBUS_SAMPLE_FS_HZ))

#define OPEN_LOOP_TS FOC_CURRENT_MEAS_PERIOD

#endif /* __COMMON_H__ */
