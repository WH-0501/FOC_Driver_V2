#ifndef __ENCODER_TYPES_H__
#define __ENCODER_TYPES_H__

#include "encoder/kth71xx.h"
#include "common.h"

/** 编码器角色 */
typedef enum
{
  ENCODER_ROLE_SHAFT = 0, ///< 电机轴端（FOC）
  ENCODER_ROLE_JOINT = 1, ///< 关节输出端（外环）
  ENCODER_ROLE_COUNT
} encoder_role_t;

typedef enum {
  ENCODER_DEV_NONE,
  ENCODER_DEV_KTH71
} encoder_dev_t;

typedef struct {
  encoder_dev_t dev;
  kth71_cs_t kth71_cs;
} encoder_binding_t;

/** shaft 编码器滤波前后快照（供 VOFA+ 等调试） */
typedef struct
{
  float theta_elec_raw_rad;
  float theta_elec_filt_rad;
  float angle_rad_raw;
  float angle_rad_filt;
  float angle_error_rad;
  float position_rad_raw;
  float position_rad_filt;
  float position_error_rad;
  float velocity_rad_s_raw;
  float velocity_rad_s_filt;
#if FOC_ENCODER_USE_PLL
  uint8_t pll_innovation_rejected;
#endif
} encoder_shaft_debug_t;

#endif /* __ENCODER_TYPES_H__ */
