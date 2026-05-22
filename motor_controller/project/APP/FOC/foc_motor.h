/**
 * @file foc_motor.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __FOC_MOTOR_H__
#define __FOC_MOTOR_H__

#include "datatypes.h"
#include "foc_math.h"

/* 16 位角度原始值 [0,65535] -> 弧度，满量程对齐 2π（与 KTH71 定义为 uint16 满刻度时一致） */
#define RAW_ANGLE_U16_TO_RAD            (_2PI * (1.0f / 65535.0f))
/* 超过此值视作跨一圈折返用于圈数校正（常量，编译期折叠） */
#define DIFF_ANGLE_WRAP_THRESH_RAD      (0.8f * _2PI)

/* 减速比 */
#define GEAR_RATIO                      (50)
/* 电机一圈的机械角度 */
#define MOTOR_ONE_REV_MECH_RAD          (2 * _PI * GEAR_RATIO)

/* 电机极对数 */
#define POLE_PAIRS                      (4)
/* 电机方向 */
#define DIR                             (1) ///< TODO: 后续优化自动辨识

void foc_motor_init(void);

/**
 * @brief 获取电机电角度
 * 
 * @return float 电机电角度
 */
void foc_get_motor_angle(void);

/**
 * @brief 获取电机速度
 * 
 * @return float 电机速度
 */
void foc_get_motor_speed(void);

/**
 * @brief 获取电机电流
 * 
 * @return float 电机电流
 */
void foc_get_motor_current(void);

#endif /* __FOC_MOTOR_H__ */