/**
 * @file foc_pid.h
 * @author your name (you@domain.com)
 * @brief This file provides the PID controller for the FOC motor control.
 * @version 0.1
 * @date 2026-05-18
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __FOC_PID_H__
#define __FOC_PID_H__

#include "error.h"
#include "datatypes.h"

typedef struct
{
  uint8_t mode;         ///< 控制模式：0 位置环, 1 速度环, 2 电流环
  float kp;             ///< 比例系数
  float ki;             ///< 积分系数
  float kd;             ///< 微分系数
  float ramp;           ///< 设定值斜坡或输出斜坡系数（依实现而定）
  float limit;          ///< 输出或积分限幅
  float last_error;     ///< 上一次误差
  float last_output;    ///< 上一次输出
  float last_integral;  ///< 上一次积分项累加
  uint32_t timestamp;   ///< 时间戳，用于积分/微分时间步. us

  float d_filter;       ///< 微分滤波系数
  float d_state;        ///< 微分状态
  float last_feedback;  ///< 上一次反馈值

  float target;         ///< 参考值
  float feedback;       ///< 反馈值
  float dt;             ///< 控制周期
} pid_t;


error_t foc_pid_init(pid_t *pid, uint8_t mode, float kp, float ki, float kd, float ramp, float limit);
error_t foc_pid_calc(pid_t *pid, float target, float feedback, float *output);
error_t foc_pid_clear(pid_t *pid);


#endif /* __FOC_PID_H__ */