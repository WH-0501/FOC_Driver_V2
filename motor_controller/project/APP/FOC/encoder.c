#include "encoder.h"
#include "common.h"
#include "foc_filter.h"
#include "foc_math.h"
#include "math_compat.h"
#include "encoder/kth71xx.h"
#include <stddef.h>

static uint32_t time_us_delta(uint32_t prev, uint32_t now)
{
  if (now >= prev)
  {
    return now - prev;
  }
  return now + 0xFFFFu - prev;
}

static int encoder_hw_read(const encoder_bind_t *bind, uint16_t *raw)
{
  kth71_cs_select_t cs;

  if ((bind == NULL) || (raw == NULL) || (bind->dev != ENCODER_DEV_KTH71))
  {
    return -1;
  }

  if (bind->kth71_cs >= KTH71_CS_COUNT)
  {
    return -1;
  }

  cs = (kth71_cs_select_t)bind->kth71_cs;
  if (KTH71_ReadAngleCs(cs, raw) == 0u)
  {
    return -1;
  }

  return 0;
}

/**
 * @brief 初始化编码器设备
 * 
 * @param dev: 编码器设备
 * @return void
 */
void encoder_dev_init_once(encoder_dev_t dev)
{
  static uint8_t s_kth71_inited = 0u;

  if ((dev == ENCODER_DEV_KTH71) && (s_kth71_inited == 0u))
  {
    KTH71_Init();
    s_kth71_inited = 1u;
  }
}

/**
 * @brief 初始化编码器观测器
 * 
 * @param obs: 编码器观测器
 * @return void
 */
void encoder_observer_init(encoder_observer_t *obs)
{
  if (obs == NULL)
  {
    return;
  }

  obs->sample.counts = 0u;
  obs->sample.angle_rad = 0.0f;
  obs->sample.position_rad = 0.0f;
  obs->sample.velocity_rad_s = 0.0f;
  obs->turn_count = 0;
  obs->prev_angle_rad = 0.0f;
  obs->prev_time_us = 0u;
  obs->first_sample = 1u;
  lpf1_init(&obs->velocity_lpf, FOC_MEAS_LPF_FC_HZ, FOC_CURRENT_LOOP_FS_HZ);
}

static float encoder_resolve_zero_rad(const encoder_bind_t *bind,
                                      encoder_role_t role,
                                      float default_zero_rad)
{
  if ((bind != NULL) && (bind->zero_rad != 0.0f))
  {
    return bind->zero_rad;
  }

  if (role == ENCODER_ROLE_SHAFT)
  {
    return default_zero_rad;
  }

  return 0.0f;
}

static float encoder_resolve_dir(const encoder_bind_t *bind, int8_t default_dir)
{
  if ((bind != NULL) && (bind->dir == 1 || bind->dir == -1))
  {
    return (bind->dir == -1) ? -1.0f : 1.0f;
  }

  return (default_dir == -1) ? -1.0f : 1.0f;
}

static uint16_t encoder_raw_to_counts(uint16_t raw_u16)
{
  return (uint16_t)((raw_u16 >> 3) & ENCODER_COUNTS_PER_REV_MASK);
}

bool encoder_observer_update(encoder_observer_t *obs,
                             const encoder_bind_t *bind,
                             encoder_role_t role,
                             float default_zero_rad,
                             int8_t default_dir,
                             uint32_t time_us)
{
  uint16_t raw_u16;
  uint16_t counts;
  float angle;
  float diff_angle;
  float delta_angle;
  float dir;
  float zero_rad;
  uint32_t delta_us;
  float velocity_rad_s;

  if ((obs == NULL) || (bind == NULL) || (bind->dev == ENCODER_DEV_NONE))
  {
    return false;
  }

  if (encoder_hw_read(bind, &raw_u16) != 0)
  {
    return false;
  }

  counts = encoder_raw_to_counts(raw_u16);
  angle = (float)counts * RAW_ANGLE_U16_TO_RAD;

  dir = encoder_resolve_dir(bind, default_dir);
  zero_rad = encoder_resolve_zero_rad(bind, role, default_zero_rad);

  diff_angle = angle - obs->prev_angle_rad;
  if (fabsf(diff_angle) > DIFF_ANGLE_WRAP_THRESH_RAD)
  {
    obs->turn_count += (diff_angle > 0.0f) ? -1 : 1;
  }

  obs->sample.counts = (uint32_t)counts;
  obs->sample.angle_rad = angle;
  obs->sample.position_rad = (float)obs->turn_count * _2PI + angle * dir - zero_rad;

  delta_angle = angle - obs->prev_angle_rad;
  if (delta_angle > _PI)
  {
    delta_angle -= _2PI;
  }
  else if (delta_angle < -_PI)
  {
    delta_angle += _2PI;
  }
  delta_angle *= dir;

  if (obs->first_sample != 0u)
  {
    obs->first_sample = 0u;
    velocity_rad_s = 0.0f;
  }
  else
  {
    delta_us = time_us_delta(obs->prev_time_us, time_us);
    if (delta_us < 1u)
    {
      delta_us = 1u;
    }
    velocity_rad_s = delta_angle / ((float)delta_us * 1.0e-6f);
  }

  obs->sample.velocity_rad_s = lpf1_update(&obs->velocity_lpf, velocity_rad_s);
  obs->prev_angle_rad = angle;
  obs->prev_time_us = time_us;
  return true;
}
