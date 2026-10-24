#include "foc_pid.h"
#include "common.h"
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
  pid->dt = PID_DT_MAX; // 默认控制周期 0.5ms. 即 20KHz 
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

  if (param->limit <= 0.0f) {
    return ERR_PARAM;
  }

  // 1. 计算 dt. 当前时间戳 - 上一次时间戳
  uint32_t current_timestamp = dwt_get_ticks_us();
  if (pid->timestamp == 0) {
    pid->timestamp = current_timestamp;
    return ERR_NONE;
  }
  pid->dt = (current_timestamp - pid->timestamp) * 1e-6f; // us to s
  if (pid->dt <= 0.0f || pid->dt > PID_DT_THRESHOLD) {
    pid->dt = PID_DT_MAX;
  }
  pid->timestamp = current_timestamp;

  // 2. 计算误差
  float error = target - feedback;

  // 3. 计算比例项
  float P = param->kp * error;

  // 4. 微分项 D + 滤波
  float D = 0.0f;
  if (param->kd != 0.0f && pid->dt > 0.0f) {
    float new_derivative = (error - pid->last_error) / pid->dt;

    // 一阶滤波，抑制噪声尖峰
    pid->d_state += param->d_filter * (new_derivative - pid->d_state);
    D = param->kd * pid->d_state;
  }
  
  // 5. 计算积分项
  float I = pid->last_integral;
  if (param->ki != 0.0f && pid->dt > 0.0f) {
    pid->last_integral += error * param->ki * pid->dt;
    I = pid->last_integral;
  }

  // 6. 计算输出 + 限幅
  float out_unlimited = P + I + D;
  if (out_unlimited > param->limit) {
    *output = param->limit;
    pid->last_integral += ( *output - out_unlimited ) * /*param->kb*/ 2.0f;
  } else if (out_unlimited < -param->limit) {
    *output = -param->limit;
    pid->last_integral += ( *output - out_unlimited ) * /*param->kb*/ 2.0f;
  } else {
    *output = out_unlimited;
  }

  // 积分兜底限幅
  if (pid->last_integral >  param->i_limit)
    pid->last_integral =  param->i_limit;
  if (pid->last_integral < -param->i_limit)
    pid->last_integral = -param->i_limit;

  // 7. 输出斜坡（限制相对上一拍的电压变化率）
  if (param->ramp > 0.0f && pid->dt > 0.0f)
  {
    float max_delta = param->ramp * pid->dt;
    float delta = *output - pid->last_output;

    if (delta > max_delta)
    {
      *output = pid->last_output + max_delta;
    }
    else if (delta < -max_delta)
    {
      *output = pid->last_output - max_delta;
    }
  }

  // 8. 更新状态
  pid->target = target;
  pid->feedback = feedback;
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
