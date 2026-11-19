#include "encoder.h"
#include "board.h"
#include "foc_filter.h"
#include "motor.h"
#include "motor_config.h"
#include <string.h>

static encoder_observer_t s_observers[ENCODER_ROLE_COUNT];
static uint32_t s_sample_time_us;

/* AT32M416 单轴板：电机轴使用 SPI2 / CS0，关节位置由电机轴换算。 */
static const encoder_binding_t s_encoder_bindings[ENCODER_ROLE_COUNT] = {
  [ENCODER_ROLE_SHAFT] = {
    .dev = ENCODER_DEV_KTH71,
    .kth71_cs = KTH71_CS_0
  },
  [ENCODER_ROLE_JOINT] = {
    .dev = ENCODER_DEV_NONE
  },
};

static bool joint_encoder_enabled(void)
{
  return s_encoder_bindings[ENCODER_ROLE_JOINT].dev != ENCODER_DEV_NONE;
}

static int32_t wrap_counts(int32_t diff)
{
  if (diff > ENCODER_COUNTS_HALF_REV) return diff - ENCODER_COUNTS_PER_REV;
  if (diff < -ENCODER_COUNTS_HALF_REV) return diff + ENCODER_COUNTS_PER_REV;
  return diff;
}

static float electrical_angle(const motor_handle_t *m, float angle)
{
  float pairs = m->config.param.pole_pairs ? (float)m->config.param.pole_pairs : 1.0f;
  return angle_normalize(angle * pairs - m->config.param.theta_elec_offset_rad);
}

void encoder_init(void)
{
  memset(s_observers, 0, sizeof(s_observers));
  s_sample_time_us = 0u;
  for (unsigned i = 0; i < ENCODER_ROLE_COUNT; ++i)
  {
    const encoder_binding_t *binding = &s_encoder_bindings[i];
    
    if (binding->dev == ENCODER_DEV_NONE) continue;

    KTH71_InitCS(binding->kth71_cs);
#if FOC_ENCODER_USE_PLL
    encoder_observer_t *u = &s_observers[i];
    u->init_count = 20u;
    float period = FOC_CURRENT_PERIOD_S * (float)(i == ENCODER_ROLE_JOINT ? FW_FOC_OUTER_LOOP_DIV : 1u);
    /* Keep the discrete PLL stable even when using a long joint sample period. */
    float bandwidth = fminf(_2PI * FOC_ENCODER_PLL_BW_HZ, 0.45f / period);
    encoder_pll_init(&u->pll, bandwidth, period);
#endif
  }
}

static bool read_angle(motor_handle_t *m, encoder_role_t role, uint16_t *raw)
{
  const encoder_binding_t *binding = &s_encoder_bindings[role];
  if (binding->dev != ENCODER_DEV_KTH71) return false;

  motor_runtime_t *rt = (FOC_PROFILE_CST_OUTER == 1) && role == ENCODER_ROLE_SHAFT ? motor_runtime(m) : NULL;
  bool profile = rt != NULL && rt->profile_cst_outer;
  uint32_t start = profile ? dwt_get_ticks() : 0u;
  bool ok = KTH71_ReadAngleCS(binding->kth71_cs, raw) != 0u;
  if (profile) rt->cst_shaft_spi_ticks = dwt_get_ticks() - start;
  return ok;
}

static int axis_direction(const motor_handle_t *m, encoder_role_t role)
{
  int dir = role == ENCODER_ROLE_JOINT && FOC_JOINT_ENCODER_DIRECTION != 0 ?
            FOC_JOINT_ENCODER_DIRECTION : m->config.param.direction;
  return dir == -1 ? -1 : 1;
}

/* Keep the observer outside the SRAM section reserved for encoder_update. */
#if defined(__CC_ARM) || defined(__GNUC__)
__attribute__((noinline))
#endif
static void update_axis_position(motor_handle_t *m, encoder_role_t role, uint32_t time_us)
{
  encoder_observer_t *u = &s_observers[role];
  encoder_sample_t *v = role == ENCODER_ROLE_SHAFT ? &m->state.shaft : &m->state.joint;
  if (!u->valid)
  {
    memset(v, 0, sizeof(*v));
    if (!joint_encoder_enabled()) m->state.joint.velocity_rad_s = 0.0f;
  }
  uint16_t raw;
  if (!read_angle(m, role, &raw)) return;

  uint16_t counts = (raw >> 3) & ENCODER_COUNTS_PER_REV_MASK;
  int32_t diff = (int32_t)counts - (int32_t)v->counts;
  int32_t step = wrap_counts(diff);
  int dir = axis_direction(m, role);
  // 电机轴偏移角
  float zero = role == ENCODER_ROLE_SHAFT ? m->config.param.theta_offset_rad : FOC_JOINT_ENCODER_ZERO_RAD;
  // 电机轴角度
  float angle = (float)counts * RAW_ANGLE_U16_TO_RAD;

#if FOC_ENCODER_USE_PLL
  /* The PLL's velocity is part of its angle predictor and runs at the sample rate.
   * Only the velocity interface below publishes speed feedback. */
  if (u->init_count)
  {
    --u->init_count;
    u->accepted_counts = counts;
    encoder_pll_reset(&u->pll, (float)counts);
    step = 0;
  }
  else
  {
    step = wrap_counts((int32_t)counts - u->accepted_counts);
    float dt = (float)(time_us - u->time_us) * 1.0e-6f;
    if (dt * u->pll.kp >= 1.0f)
    {
      /* Lost frames: re-anchor rather than extrapolate across an unstable PLL step. */
      encoder_pll_reset(&u->pll, (float)counts);
      u->pll_rejected = false;
      u->accepted_counts = counts;
    }
    else
    {
      float gate = encoder_pll_effective_gate(&u->pll, dt);
      bool rejected = gate > 0.0f && fabsf((float)step) > gate;
      if (rejected) step = 0;
      encoder_pll_update(&u->pll, (float)counts, dt,
                         (float)ENCODER_COUNTS_PER_REV, (float)ENCODER_COUNTS_HALF_REV);
      u->pll_rejected = rejected || u->pll.innovation_rejected;
      if (!rejected) u->accepted_counts = counts;
    }
    u->shadow_count += step;
  }
  v->turn_count = u->shadow_count / ENCODER_COUNTS_PER_REV;
  v->position_rad = dir * ((float)u->shadow_count * RAW_ANGLE_U16_TO_RAD - zero);
  u->raw_angle = angle_normalize(dir * angle);
  v->angle_rad = angle_normalize(dir * u->pll.pos_counts * RAW_ANGLE_U16_TO_RAD);
#else
  /* Keep the shaft's wrap threshold. Joint samples can move farther between outer ticks.使用 counts 效率比 angle 更高 */
  const int32_t turn_threshold = role == ENCODER_ROLE_JOINT && u->valid ?
                    ENCODER_COUNTS_HALF_REV : ENCODER_COUNTS_TURN_THRESH;
  if (diff > turn_threshold) --v->turn_count;
  else if (diff < -turn_threshold) ++v->turn_count;
  // 多圈机械角 = dir * (累计整圈数 * 2π + 单圈机械角 - 偏移角)
  v->position_rad = dir * ((float)v->turn_count * _2PI + angle - zero);
  v->angle_rad = dir < 0 && counts != 0 ? _2PI - angle : angle;
#endif
  /* 闭环速度只结算电机轴，关节侧不累积速度差分。 */
  if (role == ENCODER_ROLE_SHAFT)
  {
    if (u->valid)
    {
      u->pending_counts += (uint32_t)(step * dir);
    }
    else
    {
      u->velocity_time_us = time_us;
    }
  }
  u->valid = true;
  u->time_us = time_us;
  v->raw_counts = raw;
  v->counts = counts;
}

void encoder_update(motor_handle_t *m, bool outer_loop_skip)
{
  if (m == NULL) return;

  /**
   * 取 16 位计数器从上一次到现在的增量. 
   * (uint16_t) 只留低 16 位，所以减掉的是上次锁存的计数值。
   * 计数器从 65530 绕到 10，差是 16 µs，加回去得到 65546，低 16 位又等于当前硬件值 10
   * 例：
   * 上次 s_sample_time_us	本次 board_time_us()	低 16 位差	累加后
   * 0	50	50	50
   * 50	100	50	100
   * 65530	10	16	65546  // (10 - 65530) + 65536 = 16
   * 65546	60	50	65596  // (60 - 65546) + 65536 = 50
   * 以第三行为例，如果写成 s_sample_time_us = board_time_us()，值会从 65530 跳回 10，
   * 速度用的是 32 位无符号相减，10 - 65530 变成约 4294901776 µs.m
   * 
   */
#ifndef USE_VEL_PERIOD
  s_sample_time_us += (uint16_t)(board_time_us() - s_sample_time_us);
#else
  s_sample_time_us = (uint32_t)FOC_VEL_PERIOD_US;
#endif
  update_axis_position(m, ENCODER_ROLE_SHAFT, s_sample_time_us);
  m->state.theta_elec_rad = electrical_angle(m, m->state.shaft.angle_rad);

  /* 外环拍不读取关节 SPI；单编码器则用电机轴换算关节位置。 */
  if (!outer_loop_skip) return;

  /* 外环拍才读取关节 SPI；单编码器则用电机轴换算关节位置。 */
  if (joint_encoder_enabled())
  {
    update_axis_position(m, ENCODER_ROLE_JOINT, s_sample_time_us);
  }
  else
  {
    m->state.joint.counts = 0u;
    m->state.joint.turn_count = 0;
    m->state.joint.position_rad = m->state.shaft.position_rad * s_inv_gear_ratio;
    m->state.joint.angle_rad = angle_normalize(m->state.joint.position_rad);
  }
}

static bool estimate_axis_velocity(motor_handle_t *m, encoder_role_t role)
{
  encoder_observer_t *u = &s_observers[role];
  if (!u->valid) return false;

#ifndef USE_VEL_PERIOD
  uint32_t dt_us = u->time_us - u->velocity_time_us;
  if (dt_us == 0u && u->velocity_valid) return false;
#endif

  encoder_sample_t *sample = role == ENCODER_ROLE_SHAFT ? &m->state.shaft : &m->state.joint;
  int32_t delta = (int32_t)u->pending_counts;
#ifndef USE_VEL_PERIOD
  u->raw_velocity_rad_s = (dt_us && delta) ?
      (float)delta * RAW_ANGLE_U16_TO_RAD / ((float)dt_us * 1.0e-6f) : 0.0f;
#else
  u->raw_velocity_rad_s = (delta) ?
      (float)delta * RAW_ANGLE_U16_TO_RAD * FOC_INV_VEL_MEAS_PERIOD : 0.0f;
#endif
#if FOC_ENCODER_USE_PLL
  sample->velocity_rad_s = axis_direction(m, role) * u->pll.vel_counts * RAW_ANGLE_U16_TO_RAD;
#else
#ifndef USE_VEL_PERIOD
  /* 一阶低通滤波器系数 α = dt / (dt + τ) */
  float alpha = (float)dt_us / ((float)dt_us + FOC_VEL_LPF_TAU);
  sample->velocity_rad_s = lpf1_blend(alpha, u->raw_velocity_rad_s, sample->velocity_rad_s);
#else
  sample->velocity_rad_s = lpf1_blend(FOC_VEL_LPF_ALPHA, u->raw_velocity_rad_s, sample->velocity_rad_s);
#endif
#endif
  u->pending_counts = 0u;
  u->velocity_time_us = u->time_us;
  u->velocity_valid = true;
  return true;
}

bool encoder_estimate_velocity(motor_handle_t *m)
{
  if (m == NULL) return false;
  bool updated = false;

  updated = estimate_axis_velocity(m, ENCODER_ROLE_SHAFT);
  if (updated)
  {
    m->state.joint.velocity_rad_s = m->state.shaft.velocity_rad_s * s_inv_gear_ratio;
  }

  return updated;
}

bool encoder_get_shaft_debug(const motor_handle_t *m, encoder_shaft_debug_t *dbg)
{
  if (m == NULL || dbg == NULL) return false;
  static const encoder_sample_t empty = {0};
  const encoder_sample_t *shaft = s_observers[ENCODER_ROLE_SHAFT].valid ? &m->state.shaft : &empty;
  dbg->angle_rad_filt = shaft->angle_rad;
#if FOC_ENCODER_USE_PLL
  const encoder_observer_t *u = &s_observers[ENCODER_ROLE_SHAFT];
  dbg->angle_rad_raw = u->raw_angle;
  dbg->pll_innovation_rejected = u->pll_rejected;
#else
  dbg->angle_rad_raw = shaft->angle_rad;
#endif
  dbg->position_rad_raw = dbg->position_rad_filt = shaft->position_rad;
  dbg->velocity_rad_s_raw = s_observers[ENCODER_ROLE_SHAFT].raw_velocity_rad_s;
  dbg->velocity_rad_s_filt = shaft->velocity_rad_s;
  dbg->theta_elec_raw_rad = electrical_angle(m, dbg->angle_rad_raw);
  dbg->theta_elec_filt_rad = electrical_angle(m, dbg->angle_rad_filt);
  dbg->angle_error_rad = angle_delta_pm_pi(dbg->angle_rad_raw, dbg->angle_rad_filt);
  dbg->position_error_rad = 0.0f;
  return true;
}

