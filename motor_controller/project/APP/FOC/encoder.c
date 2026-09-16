#include "encoder.h"
#include "board.h"
#include "common.h"
#include "foc_filter.h"
#include "foc_math.h"
#include "math_compat.h"
#include "encoder/kth71xx.h"
#include <stddef.h>

/*
 * 编码器绑定（单编/双编）：
 * - 单编：JOINT = ENCODER_DEV_NONE，joint 由 shaft/gear_ratio 推算
 * - 双编：SHAFT 用 KTH71_CS_0，JOINT 用 KTH71_CS_1
 */
static encoder_unit_t s_encoders[ENCODER_ROLE_COUNT] = {
  [ENCODER_ROLE_SHAFT] = {
    .bind = { .dev = ENCODER_DEV_KTH71, .kth71_cs = 0 /*KTH71_CS_0*/ },
  },
  [ENCODER_ROLE_JOINT] = {
    .bind = { .dev = ENCODER_DEV_NONE },
  },
};

/**
 * @brief 确认是否启用关节编码器(低速轴编码器)
 * 
 * @return true: 启用
 * @return false: 未启用
 */
static bool encoder_joint_enabled(void)
{
  return s_encoders[ENCODER_ROLE_JOINT].bind.dev != ENCODER_DEV_NONE;
}

static void encoder_publish_state(motor_handle_t *m)
{
  const encoder_unit_t *shaft = &s_encoders[ENCODER_ROLE_SHAFT];
  const encoder_unit_t *joint = &s_encoders[ENCODER_ROLE_JOINT];
  float pole_pairs;
  float gear_ratio;
  bool shaft_ok;
  bool joint_ok;

  if (m == NULL)
  {
    return;
  }

  shaft_ok = shaft->bind.dev != ENCODER_DEV_NONE;
  joint_ok = encoder_joint_enabled();

  pole_pairs = (m->config.param.pole_pairs > 0u) ? (float)m->config.param.pole_pairs : 1.0f;
  gear_ratio = (m->config.param.gear_ratio > 0.0f) ? m->config.param.gear_ratio : 1.0f;

  if (shaft_ok)
  {
    m->state.shaft = shaft->observer.sample;
    /* shaft.angle_rad 已在控制系；乘 pole_pairs 得电角，不再重复乘 direction */
    m->state.theta_elec_rad = angle_normalize(
        (shaft->observer.sample.angle_rad * pole_pairs) -
        m->config.param.theta_elec_offset_rad);
  }

  if (joint_ok) // 启用关节编码器.则使用关节编码器观测值
  {
    m->state.joint = joint->observer.sample;
  }
  else if (shaft_ok) // 未启用关节编码器.则使用电机轴编码器观测值推算关节观测值
  {
    float joint_pos = shaft->observer.sample.position_rad / gear_ratio;
    int32_t joint_turns = (int32_t)floorf(joint_pos / _2PI);
    float joint_angle = joint_pos - (float)joint_turns * _2PI;

    m->state.joint.counts = 0u;
    m->state.joint.turn_count = joint_turns;
    m->state.joint.angle_rad = joint_angle;
    m->state.joint.position_rad = joint_pos;
    m->state.joint.velocity_rad_s = shaft->observer.sample.velocity_rad_s / gear_ratio;
  }
}

void encoder_init(void)
{
  uint8_t i;

  for (i = 0u; i < ENCODER_ROLE_COUNT; i++)
  {
    encoder_observer_init(&s_encoders[i].observer);

    if (s_encoders[i].bind.dev != ENCODER_DEV_NONE)
    {
      encoder_dev_init_once(s_encoders[i].bind.dev);
    }
  }
}

void encoder_update(motor_handle_t *m)
{
  uint8_t i;
  uint32_t now_us;
  float default_zero;
  int8_t default_dir;

  if (m == NULL)
  {
    return;
  }

  if (s_encoders[ENCODER_ROLE_SHAFT].bind.dev == ENCODER_DEV_NONE)
  {
    return;
  }

  now_us = board_time_us();
  default_zero = m->config.param.theta_offset_rad;
  default_dir = m->config.param.direction;

  for (i = 0u; i < ENCODER_ROLE_COUNT; i++)
  {
    /**
     * 双编码器更新. 若为空(如未使用输出轴编码器), 则不更新 
     */
    (void)encoder_observer_update(&s_encoders[i].observer,
                                  &s_encoders[i].bind,
                                  (encoder_role_t)i,
                                  default_zero,
                                  default_dir,
                                  now_us);
  }

  encoder_publish_state(m);
}

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
  // kth71_cs_select_t cs;

  if ((bind == NULL) || (raw == NULL) || (bind->dev != ENCODER_DEV_KTH71))
  {
    return -1;
  }

  // if (bind->kth71_cs >= KTH71_CS_COUNT)
  // {
  //   return -1;
  // }

  // cs = (kth71_cs_select_t)bind->kth71_cs;
  if (KTH71_ReadAngle(raw) == 0u)
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
  obs->sample.turn_count = 0;
  obs->sample.angle_rad = 0.0f;
  obs->sample.position_rad = 0.0f;
  obs->sample.velocity_rad_s = 0.0f;
  obs->debug.angle_rad_raw = 0.0f;
  obs->debug.position_rad_raw = 0.0f;
  obs->debug.velocity_rad_s_raw = 0.0f;
  obs->debug.prev_position_rad_raw = 0.0f;

#if FOC_ENCODER_USE_PLL
  obs->count_in_cpr = 0u;
  obs->count_in_cpr_prev = 0u;
  obs->shadow_count = 0;
  obs->init_count = 20u;
  obs->debug.pll_innovation_rejected = 0u;
  (void)encoder_pll_init(&obs->pll,
                         _2PI * FOC_ENCODER_PLL_BW_HZ,
                         FOC_CURRENT_MEAS_PERIOD);
#else
  obs->prev_angle_rad = 0.0f;
  obs->prev_time_us = 0u;
  obs->first_sample = 1u;
  (void)lpf1_init(&obs->velocity_lpf,
                  FOC_MEAS_LPF_FC_HZ,
                  FOC_CURRENT_LOOP_FS_HZ);
#endif
}

bool encoder_get_shaft_debug(const motor_handle_t *m, encoder_shaft_debug_t *dbg)
{
  const encoder_unit_t *shaft = &s_encoders[ENCODER_ROLE_SHAFT];
  float pole_pairs;
  float theta_offset;

  if ((m == NULL) || (dbg == NULL) ||
      (shaft->bind.dev == ENCODER_DEV_NONE))
  {
    return false;
  }

  pole_pairs = (m->config.param.pole_pairs > 0u) ?
                   (float)m->config.param.pole_pairs :
                   1.0f;
  theta_offset = m->config.param.theta_elec_offset_rad;

  dbg->angle_rad_raw = shaft->observer.debug.angle_rad_raw;
  dbg->angle_rad_filt = shaft->observer.sample.angle_rad;
  dbg->position_rad_raw = shaft->observer.debug.position_rad_raw;
  dbg->position_rad_filt = shaft->observer.sample.position_rad;
  dbg->velocity_rad_s_raw = shaft->observer.debug.velocity_rad_s_raw;
  dbg->velocity_rad_s_filt = shaft->observer.sample.velocity_rad_s;
  dbg->theta_elec_raw_rad = angle_normalize(
      dbg->angle_rad_raw * pole_pairs - theta_offset);
  dbg->theta_elec_filt_rad = angle_normalize(
      dbg->angle_rad_filt * pole_pairs - theta_offset);
  dbg->angle_error_rad =
      angle_delta_pm_pi(dbg->angle_rad_raw, dbg->angle_rad_filt);
  dbg->position_error_rad = dbg->position_rad_raw - dbg->position_rad_filt;
#if FOC_ENCODER_USE_PLL
  dbg->pll_innovation_rejected = shaft->observer.debug.pll_innovation_rejected;
#endif
  return true;
}

/**
 * @brief 解析编码器零位角. 
 * 
 *  当 bind.zero_rad 不为 0 时，使用 bind.zero_rad；
 *  当 role 为 ENCODER_ROLE_SHAFT 时，使用 default_zero_rad；
 *  否则返回 0.0f
 * 
 * @param bind: 编码器绑定
 * @param role: 编码器角色
 * @param default_zero_rad: 默认零位角
 * @return float: 零位角
 */
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
  return (uint16_t)((raw_u16 >> 3) & ENCODER_COUNTS_PER_REV_MASK); // 去除噪声. 保留有效位
}

#if !FOC_ENCODER_USE_PLL
static bool encoder_observer_update_legacy(encoder_observer_t *obs,
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
  float sensor_rad;

  if ((obs == NULL) || (bind == NULL) || (bind->dev == ENCODER_DEV_NONE))
  {
    return false;
  }

  if (encoder_hw_read(bind, &raw_u16) != 0)
  {
    return false;
  }

  counts = encoder_raw_to_counts(raw_u16); // 13bit 有效分辨率
  angle = (float)counts * RAW_ANGLE_U16_TO_RAD;
  dir = encoder_resolve_dir(bind, default_dir);
  zero_rad = encoder_resolve_zero_rad(bind, role, default_zero_rad);

  diff_angle = angle - obs->prev_angle_rad;
  if (fabsf(diff_angle) > DIFF_ANGLE_WRAP_THRESH_RAD)
  {
    obs->sample.turn_count += (diff_angle > 0.0f) ? -1 : 1;
  }

  sensor_rad = (float)obs->sample.turn_count * _2PI + angle;
  obs->sample.raw_counts = raw_u16;
  obs->sample.counts = counts;
  /* 观测量统一为控制坐标系：direction 在此一次性施加，turn 解包仍用 raw angle */
  obs->sample.angle_rad = angle_normalize(angle * dir);
  obs->sample.position_rad = dir * (sensor_rad - zero_rad);

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

  obs->debug.angle_rad_raw = obs->sample.angle_rad;
  obs->debug.position_rad_raw = obs->sample.position_rad;
  obs->debug.velocity_rad_s_raw = velocity_rad_s;
  obs->sample.velocity_rad_s = lpf1_update(&obs->velocity_lpf, velocity_rad_s);
  obs->prev_angle_rad = angle;
  obs->prev_time_us = time_us;
  return true;
}
#endif /* !FOC_ENCODER_USE_PLL */

#if FOC_ENCODER_USE_PLL
static int32_t encoder_wrap_delta_count(int32_t delta)
{
  const int32_t cpr = (int32_t)ENCODER_COUNTS_PER_REV;
  const int32_t cpr_half = cpr / 2;

  if (delta > cpr_half)
  {
    delta -= cpr;
  }
  else if (delta < -cpr_half)
  {
    delta += cpr;
  }
  return delta;
}

static void encoder_publish_sample_pll(encoder_observer_t *obs,
                                       uint16_t counts_raw,
                                       float dir,
                                       float zero_rad)
{
  float count_to_rad = RAW_ANGLE_U16_TO_RAD;
  float sensor_rad;
  float angle_meas_rad;
  float angle_pll_rad;

  sensor_rad = (float)obs->shadow_count * count_to_rad;
  angle_meas_rad = (float)obs->count_in_cpr * count_to_rad;
  angle_pll_rad = obs->pll.pos_counts * count_to_rad;

  obs->sample.counts = (uint32_t)counts_raw;
  obs->sample.turn_count = obs->shadow_count / (int32_t)ENCODER_COUNTS_PER_REV;
  obs->sample.angle_rad = angle_normalize(angle_pll_rad * dir);
  obs->sample.position_rad = dir * (sensor_rad - zero_rad);
  obs->sample.velocity_rad_s = dir * obs->pll.vel_counts * count_to_rad;

  obs->debug.angle_rad_raw = angle_normalize(angle_meas_rad * dir);
  obs->debug.position_rad_raw = dir * (sensor_rad - zero_rad);
}

static bool encoder_observer_update_pll(encoder_observer_t *obs,
                                        const encoder_bind_t *bind,
                                        encoder_role_t role,
                                        float default_zero_rad,
                                        int8_t default_dir,
                                        uint32_t time_us)
{
  uint16_t raw_u16;
  uint16_t counts_raw;
  uint16_t count_in_cpr;
  int32_t delta_count;
  float dir;
  float zero_rad;
  float cpr_f;
  float cpr_half;
  float dt_s;

  (void)time_us;

  if ((obs == NULL) || (bind == NULL) || (bind->dev == ENCODER_DEV_NONE))
  {
    return false;
  }

  if (encoder_hw_read(bind, &raw_u16) != 0)
  {
    return false;
  }

  counts_raw = encoder_raw_to_counts(raw_u16);
  count_in_cpr = counts_raw;
  dir = encoder_resolve_dir(bind, default_dir);
  zero_rad = encoder_resolve_zero_rad(bind, role, default_zero_rad);
  cpr_f = (float)ENCODER_COUNTS_PER_REV;
  cpr_half = cpr_f * 0.5f;
  dt_s = FOC_CURRENT_MEAS_PERIOD;

  if (obs->init_count > 0u)
  {
    obs->init_count--;
    obs->count_in_cpr = count_in_cpr;
    obs->count_in_cpr_prev = count_in_cpr;
    obs->shadow_count = 0;
    encoder_pll_reset(&obs->pll, (float)count_in_cpr);
    encoder_publish_sample_pll(obs, counts_raw, dir, zero_rad);
    return true;
  }

  delta_count =
      encoder_wrap_delta_count((int32_t)count_in_cpr - (int32_t)obs->count_in_cpr_prev);

  {
    float innov_gate = encoder_pll_effective_gate(&obs->pll, dt_s);
    int32_t abs_delta = delta_count;
    uint8_t shadow_rejected = 0u;

    if (abs_delta < 0)
    {
      abs_delta = -abs_delta;
    }
    if ((innov_gate > 0.0f) && ((float)abs_delta > innov_gate))
    {
      shadow_rejected = 1u;
    }
    else
    {
      obs->shadow_count += delta_count;
      obs->count_in_cpr_prev = count_in_cpr;
    }

    obs->count_in_cpr = count_in_cpr;

    encoder_pll_update(&obs->pll,
                       (float)count_in_cpr,
                       dt_s,
                       cpr_f,
                       cpr_half);
    obs->debug.pll_innovation_rejected =
        (uint8_t)(shadow_rejected | obs->pll.innovation_rejected);
  }

  encoder_publish_sample_pll(obs, counts_raw, dir, zero_rad);
  obs->debug.velocity_rad_s_raw =
      dir * (float)delta_count * RAW_ANGLE_U16_TO_RAD / dt_s;
  obs->debug.prev_position_rad_raw = obs->debug.position_rad_raw;
  return true;
}
#endif /* FOC_ENCODER_USE_PLL */

bool encoder_observer_update(encoder_observer_t *obs,
                             const encoder_bind_t *bind,
                             encoder_role_t role,
                             float default_zero_rad,
                             int8_t default_dir,
                             uint32_t time_us)
{
#if FOC_ENCODER_USE_PLL
  return encoder_observer_update_pll(obs,
                                     bind,
                                     role,
                                     default_zero_rad,
                                     default_dir,
                                     time_us);
#else
  return encoder_observer_update_legacy(obs,
                                        bind,
                                        role,
                                        default_zero_rad,
                                        default_dir,
                                        time_us);
#endif
}
