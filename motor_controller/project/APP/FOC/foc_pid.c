#include "foc_pid.h"
#include "math_compat.h"
#include "dwt_profile_delay.h"
#include <math.h>
#include <stddef.h>

error_t foc_pid_init(pid_state_t *pid, uint8_t mode)
{
  if (pid == NULL) {
    return ERR_NULL;
  }

  pid->mode = mode;

  pid->last_error = 0.0f;
  pid->last_output = 0.0f;
  pid->last_integral = 0.0f;
  pid->d_state = 0.0f;
  pid->last_feedback = 0.0f;
  pid->target = 0.0f;
  pid->feedback = 0.0f;
  pid->dt = 0.5f; // 默认控制周期 0.5ms. 即 20KHz 
  pid->timestamp = dwt_get_ticks_us();
  return ERR_NONE;
}

/**
 * @brief 
 * 
 * @param pid PID 控制器
 * @param target 目标值
 * @param feedback 反馈值
 * @param output 输出值
 * @return 错误码
 */
error_t foc_pid_calc(pid_state_t *pid, const pid_param_t *param, float target, float feedback, float *output)
{
  if ((pid == NULL) || (param == NULL) || (output == NULL)) {
    return ERR_NULL;
  }

  // 1. 计算 dt. 当前时间戳 - 上一次时间戳
  uint32_t current_timestamp = dwt_get_ticks_us();
  if (pid->timestamp == 0) {
    pid->timestamp = current_timestamp;
    return ERR_NONE;
  }
  pid->dt = (current_timestamp - pid->timestamp) * 0.000001f; // us to s
  pid->timestamp = current_timestamp;

  // 2. 计算误差
  float error = target - feedback;

  // 3. 计算比例项
  float P = param->kp * error;
  
  // 4. 计算积分项
  float I = 0.0f;
  if (param->ki != 0.0f && pid->dt > 0.0f) {
    float new_integral = pid->last_integral + error * param->ki * pid->dt;

    // 积分限幅
    if (new_integral > param->limit) {
      new_integral = param->limit;
    } else if (new_integral < -param->limit) {
      new_integral = -param->limit;
    }
    
    // 积分抗饱和
    float pre_out = P + new_integral;
    if (fabsf(pre_out) <= param->limit) {
        pid->last_integral = new_integral;
    }
    I = pid->last_integral;
  }

  // 5. 微分项 D + 滤波
  float D = 0.0f;
  if (param->kd != 0.0f && pid->dt > 0.0f) {
    float new_derivative = (error - pid->last_error) / pid->dt;

    // 一阶滤波，抑制噪声尖峰
    pid->d_state += param->d_filter * (new_derivative - pid->d_state);
    D = param->kd * pid->d_state;
  }

  // 6. 计算输出 + 限幅
  *output = P + I + D;
  if (*output > param->limit) {
    *output = param->limit;
  } else if (*output < -param->limit) {
    *output = -param->limit;
  }

  // 7. 输出斜坡
  if (param->ramp > 0.0f && pid->dt > 0.0f) {
    float ramp_output = pid->last_output + param->ramp * pid->dt;
    if (ramp_output > *output) {
      *output = ramp_output;
    } else if (ramp_output < *output) {
      *output = ramp_output;
    }
  }

  // 8. 更新状态
  pid->last_error = error;
  pid->last_output = *output;

  return ERR_NONE;
}

error_t foc_pid_clear(pid_state_t *pid)
{   
    if (pid == NULL) {
        return ERR_NULL;
    }
    // pid->target = 0.0f;
    // pid->feedback = 0.0f;
    // pid->output = 0.0f;
    pid->last_error = 0.0f;
    pid->last_output = 0.0f;
    pid->last_integral = 0.0f;
    pid->d_state = 0.0f;
    pid->last_feedback = 0.0f;
    pid->timestamp = 0;
    return ERR_NONE;
}
