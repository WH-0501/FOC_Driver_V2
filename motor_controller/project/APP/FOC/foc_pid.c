#include "foc_pid.h"
#include "common.h"
#include "math_compat.h"
#include "dwt_profile_delay.h"
#include <stddef.h>
#include <math.h>
#include <float.h>

error_t foc_pid_init(pid_state_t *pid, const uint8_t mode)
{
  if (pid == NULL)
  {
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
  pid->dt = (mode == FOC_PID_LOOP_CURRENT) ? FOC_CURRENT_PERIOD_S :
                        FW_FOC_OUTER_LOOP_DIV * FOC_CURRENT_PERIOD_S;
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
  if ((pid == NULL) || (param == NULL) || (output == NULL))
  {
    return ERR_NULL;
  }

  if (param->limit <= FLT_EPSILON ||
      !(param->i_limit >= FLT_EPSILON && param->i_limit <= FLT_MAX))
  {
    return ERR_PARAM;
  }

  if (param->i_limit == 0.0f)
  {
    pid->last_integral = 0.0f;
  }

  /* The PID advances only when its loop executes. Timestamp units are microseconds. */
  pid->timestamp += (uint32_t)(pid->dt * 1.0e6f + 0.5f);

  float error = target - feedback;
  float P = param->kp * error;

  float D = 0.0f;
  if (param->kd > FLT_EPSILON && pid->dt > FLT_EPSILON)
  {
    float new_derivative = (error - pid->last_error) / pid->dt;

    // 一阶低通滤波, 抑制尖峰噪声
    pid->d_state += param->d_filter * (new_derivative - pid->d_state);
    D = param->kd * pid->d_state;
  }

  float ki_eff = param->ki;
  if (pid->mode == FOC_PID_LOOP_CURRENT)
  {
    ki_eff *= FOC_PID_CUR_KI_SCALE;
  }
  float I = pid->last_integral;
  if (ki_eff != 0.0f && param->i_limit > 0.0f && pid->dt > 0.0f)
  {
    pid->last_integral += error * ki_eff * pid->dt;
    /* Enforce the integral contribution budget before using it in output. */
    pid->last_integral = CLAMP(pid->last_integral, -param->i_limit, param->i_limit);
    I = pid->last_integral;
  }

  float out_unlimited = P + I + D;
  if (out_unlimited > param->limit)
  {
    *output = param->limit;
    pid->last_integral += (*output - out_unlimited) * /*param->kb*/ 2.0f;
  }
  else if (out_unlimited < -param->limit)
  {
    *output = -param->limit;
    pid->last_integral += (*output - out_unlimited) * /*param->kb*/ 2.0f;
  }
  else
  {
    *output = out_unlimited;
  }

  // 积分限幅
  if (pid->last_integral > param->i_limit)
  {
    pid->last_integral = param->i_limit;
  }
  if (pid->last_integral < -param->i_limit)
  {
    pid->last_integral = -param->i_limit;
  }

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

error_t foc_pid_calc_iq_current(pid_state_t *pid, const pid_param_t *param, float target, float feedback,
                                float *output)
{
  float error;
  float P;
  float out_unlimited;
  float ki_dt;
  float limit;

  if (pid == NULL || param == NULL || output == NULL)
  {
    return ERR_NULL;
  }

  if (param->limit <= FLT_EPSILON)
  {
    return ERR_PARAM;
  }

  pid->timestamp += (uint32_t)(pid->dt * 1.0e6f + 0.5f);

  error = target - feedback;
  P = param->kp * error;

  ki_dt = param->ki * FOC_PID_CUR_KI_SCALE * pid->dt;
  if (ki_dt != 0.0f && param->i_limit > 0.0f)
  {
    pid->last_integral += error * ki_dt;
    if (pid->last_integral > param->i_limit)
    {
      pid->last_integral = param->i_limit;
    }
    else if (pid->last_integral < -param->i_limit)
    {
      pid->last_integral = -param->i_limit;
    }
  }

  out_unlimited = P + pid->last_integral;
  limit = param->limit;
  if (out_unlimited > limit)
  {
    *output = limit;
    pid->last_integral += (*output - out_unlimited) * 2.0f;
  }
  else if (out_unlimited < -limit)
  {
    *output = -limit;
    pid->last_integral += (*output - out_unlimited) * 2.0f;
  }
  else
  {
    *output = out_unlimited;
  }

  pid->target = target;
  pid->feedback = feedback;
  pid->last_error = error;
  pid->last_output = *output;
  return ERR_NONE;
}

error_t foc_pid_clear(pid_state_t *pid)
{
  if (pid == NULL)
  {
    return ERR_NULL;
  }
  pid->last_error = 0.0f;
  pid->last_output = 0.0f;
  pid->last_integral = 0.0f;
  pid->d_state = 0.0f;
  pid->last_feedback = 0.0f;
  pid->timestamp = 0;
  return ERR_NONE;
}
