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
/**
 * @brief 电流环与测量链路的采样率/滤波配置
 *        1. FOC_CURRENT_LOOP_FS_HZ: 电流环采样率
 *        2. FOC_MEAS_LPF_FC_HZ: 电流环滤波器截止频率
 *        3. FOC_CURRENT_MEAS_PERIOD: 电流环周期
 */
#ifndef FOC_CURRENT_LOOP_FS_HZ
#define FOC_CURRENT_LOOP_FS_HZ (FOC_PWM_HZ) // 当前 20KHz
#endif
#ifndef FOC_MEAS_LPF_FC_HZ
#define FOC_MEAS_LPF_FC_HZ (FOC_CURRENT_LOOP_FS_HZ / 10u)
#endif
#ifndef FOC_CURRENT_MEAS_PERIOD
#define FOC_CURRENT_MEAS_PERIOD (float)(1.0f / (float)FOC_CURRENT_LOOP_FS_HZ)
#endif

#ifndef FOC_CURRENT_BANDWIDTH_HZ
#define FOC_CURRENT_BANDWIDTH_HZ (800.0f)
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

/* 开环周期 */
#define OPEN_LOOP_TS         FOC_CURRENT_MEAS_PERIOD

/** 
 * @brief 母线电压采样率 [Hz]（慢路径，与电流环解耦）
 * 1. 母线电压 LPF 截止 [Hz]（须 < FOC_VBUS_SAMPLE_FS_HZ/2，否则 lpf1_init 失败）
 * 2. 母线电压滤波只是让电压波形看起来更平滑，真正决定 SVPWM 精度的是母线电容
 * 3. 可以考虑直接降低采样频次达到一样的效果（如降低到 100Hz）
 * 4. 或者使用多次采样平均值作为母线电压值（如 4 次采样平均）
 */
#ifndef FOC_VBUS_SAMPLE_FS_HZ
#define FOC_VBUS_SAMPLE_FS_HZ (100.0f)  ///< fs
#endif
#ifndef FOC_VBUS_LPF_FC_HZ
#define FOC_VBUS_LPF_FC_HZ (10.0f)  ///< fc
#endif
#ifndef FOC_VBUS_POLL_MIN_INTERVAL_US
#define FOC_VBUS_POLL_MIN_INTERVAL_US ((uint32_t)(1000000.0f / FOC_VBUS_SAMPLE_FS_HZ))
#endif

/* 机械 100RPM → 机械角速度 [rad/s] */
#define OL_OMEGA_100RPM_MECH   RPM_TO_RAD_PER_SEC(100.0f)

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
#define DEF_D_KI (1507.96f)
#define DEF_D_KD (0.0f)
#define DEF_D_RAMP (0.0f)
#define DEF_D_LIMIT (0.0f)
#define DEF_D_D_FILTER (0.0f)
#define DEF_D_ANTI_WINDUP_GAIN (0.5f)
#define DEF_Q_ANTI_WINDUP_GAIN (0.5f)
#define DEF_I_LIMIT_FACTOR (0.85f)

#define PID_DT_THRESHOLD (0.5f)
#define PID_DT_MAX (0.001f)

#endif /* __COMMON_H__ */
