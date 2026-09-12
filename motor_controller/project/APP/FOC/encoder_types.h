#ifndef __ENCODER_TYPES_H__
#define __ENCODER_TYPES_H__

#include <stdint.h>
#include "datatypes.h"
#include "foc_filter.h"

/** 编码器角色 */
typedef enum
{
  ENCODER_ROLE_SHAFT = 0, ///< 电机轴端（FOC）
  ENCODER_ROLE_JOINT = 1, ///< 关节输出端（外环）
} encoder_role_t;

#define ENCODER_ROLE_COUNT 2u

/** 编码器芯片类型；未安装填 ENCODER_DEV_NONE */
typedef enum
{
  ENCODER_DEV_NONE = 0,
  ENCODER_DEV_KTH71,
} encoder_dev_t;

/**
 * 零偏约定（与 motor_param_t 配合）：
 * - SHAFT：bind.zero_rad 为 0 时使用 motor_param.theta_offset_rad
 * - JOINT：仅使用 bind.zero_rad（双编码器关节零位）
 * - motor_param.theta_elec_offset_rad：FOC 电角零位，与机械零偏独立
 */
typedef struct
{
  encoder_dev_t dev;
  float zero_rad;
  int8_t dir;      ///< 控制正方向：1 / -1；0 表示沿用 motor_param.direction（观测发布时施加）
  uint8_t kth71_cs; ///< dev==ENCODER_DEV_KTH71 时片选索引（见 kth71_cs_select_t）
} encoder_bind_t;

typedef struct
{
  encoder_sample_t sample;
  float prev_angle_rad;
  uint32_t prev_time_us;
  uint8_t first_sample;
  lpf1_t velocity_lpf;
} encoder_observer_t;

typedef struct
{
  encoder_bind_t bind;
  encoder_observer_t observer;
} encoder_unit_t;

#endif /* __ENCODER_TYPES_H__ */
