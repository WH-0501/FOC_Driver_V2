/**
 * @file foc_pid.h
 * @brief FOC 三环 PID：运行期增益在 pid_param_t，环状态在 pid_state_t。
 */

#ifndef __FOC_PID_H__
#define __FOC_PID_H__

#include <stdint.h>
#include "error_types.h"

/** 电流环 config/CAN 中 Ki 为 Ki_phys / FOC_PID_CUR_KI_SCALE；环内积分还原为物理 Ki。 */
#define FOC_PID_CUR_KI_SCALE (10000.0f)

#define FOC_PID_LOOP_POSITION 0u
#define FOC_PID_LOOP_VELOCITY 1u
#define FOC_PID_LOOP_CURRENT  2u

typedef struct
{
  float kp;             ///< 比例系数
  float ki;             ///< 积分系数（电流环 Id/Iq 为缩放值，见 FOC_PID_CUR_KI_SCALE）
  float kd;             ///< 微分系数
  float ramp;           ///< 设定值斜坡或输出斜坡系数（依实现而定）
  float limit;          ///< 输出或积分限幅
  float i_limit;        ///< 积分输出上限，单位同本环输出；0 明确禁用积分，必须为有限非负值
  float anti_windup_gain;///< 反算系数，建议 0.5f ~ 1.0f
  float d_filter;       ///< 微分滤波系数
} pid_param_t;

typedef struct
{
  uint8_t mode;         ///< 控制模式：0 位置环, 1 速度环, 2 电流环
  float last_error;     ///< 上一次误差
  float last_output;    ///< 上一次输出
  float last_integral;  ///< 上一次积分项累加
  uint32_t timestamp;   ///< 时间戳，用于积分/微分时间步. us
  float d_state;        ///< 微分状态
  float last_feedback;  ///< 上一次反馈值
  float target;         ///< 参考值
  float feedback;       ///< 反馈值
  float dt;             ///< 控制周期
} pid_state_t;

error_t foc_pid_init(pid_state_t *pid, const uint8_t mode);
error_t foc_pid_calc(pid_state_t *pid, const pid_param_t *param, float target, float feedback, float *output);
/** 电流环 ISR：假定 kd/ramp=0、param 已校验；无 timestamp/CAN 观测字段更新 */
error_t foc_pid_calc_iq_current(pid_state_t *pid, const pid_param_t *param, float target, float feedback,
                                float *output);
error_t foc_pid_clear(pid_state_t *pid);

#endif /* __FOC_PID_H__ */
