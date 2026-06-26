#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include "foc_math.h"
#include "math_compat.h"
#include "dwt_profile_delay.h"

#define BUS_VOLTAGE     (24.0f) ///< 母线电压
#define POLE_PAIRS      (5) ///< 极对数
#define GEAR_RATIO      (50) ///< 电机到关节减速比（直驱填 1.0）
#define ENCODER_COUNTS_PER_REV (4096) ///< 机械旋转一圈编码器计数（分辨率定义依传感器）
#define THETA_ELEC_OFFSET_RAD (0.0f) ///< 电角度零偏置 [rad]，对齐/校准后写入
#define THETA_OFFSET_RAD (0.0f) ///< 机械角度零偏置 [rad]，对齐/校准后写入

/* 速度单位换算（内部统一使用 rad/s） */
#define RPM_TO_RAD_PER_SEC(rpm) ((float)(rpm) * (_2PI / 60.0f))
#define RAD_PER_SEC_TO_RPM(rad_s) ((float)(rad_s) * (60.0f / _2PI))

/* 限制参数 */
#define MAX_VELOCITY_RPM   (3000.0f)                         ///< 最大速度 [RPM]
#define MAX_VELOCITY_RAD_S (RPM_TO_RAD_PER_SEC(MAX_VELOCITY_RPM)) ///< 最大速度 [rad/s]
#define MAX_ACC_RAD_S2 (100.0f) ///< 最大加速度 [rad/s²]
#define MAX_JERK_RAD_S3 (1000.0f) ///< 最大加加速度 [rad/s³]



/* 16 位角度原始值 [0,65535] -> 弧度，满量程对齐 2π（与 KTH71 定义为 uint16 满刻度时一致） */
#define RAW_ANGLE_U16_TO_RAD            (_2PI * (1.0f / 65535.0f))
#define DIFF_ANGLE_WRAP_THRESH_RAD      (0.8f * _2PI) // 超过此值视作跨一圈折返用于圈数校正


/* 电角度对齐时的电压参数 */
#define ALIGN_ELECTRICAL_VOLTAGE_D_V   (0.0f)
#define ALIGN_ELECTRICAL_VOLTAGE_Q_V   (2.0f)



/* 外环相对电流环分频；PID_POS.dt / PID_VEL.dt 应对应该实际周期（约 电流周期×本系数）*/
#define FOC_OUTER_LOOP_DIV   15u
#define FOC_PWM_HZ           30000u
#define FOC_PWM_PERIOD_TS    (1.0f / (float)FOC_PWM_HZ)
#define OPEN_LOOP_TS         ((float)FOC_OUTER_LOOP_DIV / (float)FOC_PWM_HZ)

#define OPEN_LOOP_UQ_RAMP_RATE      20.0f
#define OPEN_LOOP_OMEGA_RAMP_RATE   20.0f
#define OPEN_LOOP_UQ_MAX            0.2f

/**
 * @brief 电流环与测量链路的采样率/滤波配置
 */
#ifndef FOC_CURRENT_LOOP_FS_HZ
#define FOC_CURRENT_LOOP_FS_HZ (20000.0f)
#endif
#ifndef FOC_MEAS_LPF_FC_HZ
#define FOC_MEAS_LPF_FC_HZ (2000.0f)
#endif
#ifndef FOC_CURRENT_MEAS_PERIOD
#define FOC_CURRENT_MEAS_PERIOD (float)(1.0f / (float)FOC_CURRENT_LOOP_FS_HZ)
#endif

/* 机械 100RPM → 机械角速度 [rad/s] */
#define OL_OMEGA_100RPM_MECH   RPM_TO_RAD_PER_SEC(100.0f)

#define DEF_UVLO_VOLTAGE (10.0f)
#define DEF_OVP_VOLTAGE (24.0f)
#define DEF_OCP_CURRENT (1.0f)
#define DEF_TEMP_LIMIT (100.0f)
#define DEF_SW_OCP (1.2f)
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
#define DEF_Q_KP (1.8f)
#define DEF_Q_KI (0.22f)
#define DEF_Q_KD (0.0f)
#define DEF_Q_RAMP (0.0f)
#define DEF_Q_LIMIT (0.0f)
#define DEF_Q_D_FILTER (0.0f)
#define DEF_D_KP (0.0f)
#define DEF_D_KI (0.0f)
#define DEF_D_KD (0.0f)
#define DEF_D_RAMP (0.0f)
#define DEF_D_LIMIT (0.0f)
#define DEF_D_D_FILTER (0.0f)

#endif /* __COMMON_H__ */
