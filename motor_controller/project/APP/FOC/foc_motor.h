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
