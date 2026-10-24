#include "foc_calibration.h"
#include "foc.h"
#include "foc_sense.h"
#include "current_sense.h"
#include "foc_fault.h"
#include "foc_pid.h"
#include "motor.h"
#include "common.h"
#include "math_compat.h"
#include <string.h>

typedef enum
{
  CAL_LOCK_AXIS_UD = 0,
  CAL_LOCK_AXIS_UQ,
} cal_lock_axis_t;

typedef struct
{
  motor_handle_t *m;
  motor_runtime_t *rt;
  cal_elec_align_cfg_t cfg;
  cal_elec_align_method_t method;
  float align_angle;
  float lock_ud;
  float lock_uq;
  int8_t dir;
  float raw_locked;
  float offset_rad;
} cal_ctx_t;

void cal_elec_align_cfg_default(cal_elec_align_cfg_t *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  memset(cfg, 0, sizeof(*cfg));
  cfg->align_angle_rad = ALIGN_ELECTRICAL_ANGLE_RAD;
  cfg->voltage_d_v = ALIGN_ELECTRICAL_VOLTAGE_D_V;
  cfg->voltage_q_v = ALIGN_ELECTRICAL_VOLTAGE_Q_V;
  cfg->hold_ms = ALIGN_HOLD_MS;
  cfg->sample_count = (uint8_t)ALIGN_SAMPLE_COUNT;
  cfg->sample_interval_ms = ALIGN_SAMPLE_INTERVAL_MS;
  cfg->relock_ms = ALIGN_RELOCK_MS;
  cfg->polarity_test_uq_v = ALIGN_POLARITY_TEST_UQ_V;
  cfg->polarity_test_ms = ALIGN_POLARITY_TEST_MS;
  cfg->polarity_iq_min_a = ALIGN_POLARITY_IQ_MIN_A;
  cfg->rotor_engage_tol_rad = ALIGN_ROTOR_ENGAGE_TOL_RAD;
  cfg->rotor_spread_max_rad = ALIGN_ROTOR_SPREAD_MAX_RAD;
  cfg->dual_point_delta_rad = _PI;
  cfg->uq_d_axis_offset_rad = -_PI * 0.5f;
  cfg->hfi_voltage_v = 2.0f;
  cfg->hfi_freq_hz = 400.0f;
  cfg->hfi_cycles = 8u;
  cfg->hfi_scan_steps = 36u;
}

/** 与旧 foc_align_raw_elec_rad / encoder 一致：angle_rad × pole_pairs，未减 offset */
static float cal_encoder_raw_elec_rad(const motor_handle_t *m)
{
  float pole_pairs;

  if (m == NULL)
  {
    return 0.0f;
  }

  pole_pairs = (m->config.param.pole_pairs > 0u) ? (float)m->config.param.pole_pairs : 1.0f;
  return m->state.shaft.angle_rad * pole_pairs;
}

static void cal_flip_direction(cal_ctx_t *ctx)
{
  if ((ctx == NULL) || (ctx->m == NULL) || (ctx->rt == NULL))
  {
    return;
  }

  if (ctx->m->config.param.direction == DIR_CCW)
  {
    ctx->m->config.param.direction = DIR_CW;
  }
  else
  {
    ctx->m->config.param.direction = DIR_CCW;
  }
  ctx->dir = ctx->m->config.param.direction;
  ctx->rt->align_direction_flipped = true;
}

/**
 * MP6540 相电流极性取反后：+Vq 探针下方向正确时 i_q 为负，错误时为正。
 * 与旧版（极性 +1）下「iq<0 则翻转」在物理上等价。
 */
static bool cal_polarity_probe_needs_flip(float iq_probe, float iq_min_a)
{
  if (fabsf(iq_probe) < iq_min_a)
  {
    return false;
  }

  // if (CURRENT_SENSE_PHASE_POLARITY < 0.0f)
  // {
  //   return (iq_probe > 0.0f);
  // }

  // return (iq_probe < 0.0f);
  return (iq_probe > 0.0f);
}

static bool cal_polarity_probe_aligned(float iq_probe, float iq_min_a)
{
  if (fabsf(iq_probe) < iq_min_a)
  {
    return true;
  }

  // if (CURRENT_SENSE_PHASE_POLARITY < 0.0f)
  // {
  //   return (iq_probe < 0.0f);
  // }

  // return (iq_probe > 0.0f);
  return (iq_probe < 0.0f);
}

static void cal_hold_voltage(cal_ctx_t *ctx, float ud, float uq, float angle_el, uint32_t hold_ms)
{
  uint32_t elapsed = 0u;

  if ((ctx == NULL) || (ctx->m == NULL))
  {
    return;
  }

  while (elapsed < hold_ms)
  {
    foc_voltage(ctx->m, uq, ud, angle_el);
    delay_ms(ctx->cfg.sample_interval_ms);
    elapsed += ctx->cfg.sample_interval_ms;
  }
}

static float cal_sample_raw_at_lock(cal_ctx_t *ctx,
                                    float ud,
                                    float uq,
                                    float angle_el,
                                    float *spread_out)
{
  float sum = 0.0f;
  float raw;
  float raw_min = 1e9f;
  float raw_max = -1e9f;
  uint8_t count = ctx->cfg.sample_count;

  if (spread_out != NULL)
  {
    *spread_out = 0.0f;
  }
  if ((ctx == NULL) || (ctx->m == NULL) || (count == 0u))
  {
    return 0.0f;
  }

  for (uint8_t i = 0u; i < count; i++)
  {
    foc_voltage(ctx->m, uq, ud, angle_el);
    foc_sense_update(ctx->m);
    raw = cal_encoder_raw_elec_rad(ctx->m);
    sum += raw;
    if (raw < raw_min)
    {
      raw_min = raw;
    }
    if (raw > raw_max)
    {
      raw_max = raw;
    }
    delay_ms(ctx->cfg.sample_interval_ms);
  }

  if (spread_out != NULL)
  {
    float range = raw_max - raw_min;
    if (range > _PI)
    {
      range = _2PI - range;
    }
    *spread_out = range;
  }

  return sum / (float)count;
}

static float cal_probe_iq(cal_ctx_t *ctx, float uq, float ud, float angle_el, uint16_t hold_ms)
{
  if (ctx == NULL) return 0.0f;

  cal_hold_voltage(ctx, ud, uq, angle_el, hold_ms);
  foc_sense_update(ctx->m);
  return ctx->m->state.i_q;
}

/** Uq 锁定极性探针：保持 lock_uq，叠加 Ud 脉冲，读 i_d */
static float cal_probe_id_uq_lock(cal_ctx_t *ctx,
                                  float ud_pulse,
                                  float uq_lock,
                                  float angle_el,
                                  uint16_t hold_ms)
{
  if (ctx == NULL) return 0.0f;

  cal_hold_voltage(ctx, ud_pulse, uq_lock, angle_el, hold_ms);
  foc_sense_update(ctx->m);
  return ctx->m->state.i_d;
}

static bool cal_spread_ok(const cal_ctx_t *ctx, float spread)
{
  return (ctx != NULL) && (spread <= ctx->cfg.rotor_spread_max_rad);
}

static float cal_offset_friction_avg(float off0, float off1)
{
  return angle_normalize(0.5f * (off0 + off1));
}

static cal_lock_axis_t cal_ctx_lock_axis(const cal_ctx_t *ctx)
{
  if (ctx == NULL)
  {
    return CAL_LOCK_AXIS_UD;
  }

  switch (ctx->method)
  {
    case CAL_ELEC_ALIGN_UQ:
    case CAL_ELEC_ALIGN_DUAL_POINT_UQ:
      return CAL_LOCK_AXIS_UQ;

    case CAL_ELEC_ALIGN_UD:
    case CAL_ELEC_ALIGN_DUAL_POINT:
    case CAL_ELEC_ALIGN_HFI:
    default:
      return CAL_LOCK_AXIS_UD;
  }
}

/**
 * @brief 开环 Park 角 θ_open 下，转子 d 轴期望电角（编码器 raw 电角系）。
 * Ud 锁定：θ_d ≈ θ_open；Uq 锁定：θ_d ≈ θ_open + uq_d_axis_offset_rad。
 */
static float cal_rotor_d_axis_ref_rad(const cal_ctx_t *ctx, float open_loop_rad)
{
  if (cal_ctx_lock_axis(ctx) == CAL_LOCK_AXIS_UQ)
  {
    return angle_normalize(open_loop_rad + ctx->cfg.uq_d_axis_offset_rad);
  }
  return angle_normalize(open_loop_rad);
}

/**
 * Ud / Ud双点 / HFI：offset = raw − θ_d_ref（d 轴锁定，θ_d_ref = θ_open）。
 * Uq双点：offset = raw − θ_d_ref（q 轴锁定，θ_d_ref = θ_open + uq_d_axis_offset）。
 */
static float cal_offset_from_sample(const cal_ctx_t *ctx, float raw, float open_loop_rad)
{
  return angle_normalize(raw - cal_rotor_d_axis_ref_rad(ctx, open_loop_rad));
}

/** Uq 单点（dex-con）：offset = raw，对齐后 theta_elec ≈ 0。与 Ud 公式不同，勿混比数值。 */
static float cal_offset_uq_dex(float raw)
{
  return angle_normalize(raw);
}

static float cal_offset_dual_friction_avg(const cal_ctx_t *ctx,
                                          float raw0,
                                          float open_loop0,
                                          float raw1,
                                          float open_loop1)
{
  float off0 = cal_offset_from_sample(ctx, raw0, open_loop0);
  float off1 = cal_offset_from_sample(ctx, raw1, open_loop1);
  return cal_offset_friction_avg(off0, off1);
}

static void cal_setup_lock_axes(cal_ctx_t *ctx)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->align_angle = ctx->cfg.align_angle_rad; // 针对 HFI 方案,扫角后会改写 align_angle, 因此不能直接替换

  switch (ctx->method)
  {
    case CAL_ELEC_ALIGN_UQ:
    case CAL_ELEC_ALIGN_DUAL_POINT_UQ:
      ctx->lock_ud = 0.0f;
      ctx->lock_uq = ctx->cfg.voltage_q_v;
      break;

    case CAL_ELEC_ALIGN_UD:
    case CAL_ELEC_ALIGN_DUAL_POINT:
    default:
      ctx->lock_ud = ctx->cfg.voltage_d_v;
      ctx->lock_uq = 0.0f;
      break;

    case CAL_ELEC_ALIGN_HFI:
      ctx->lock_ud = 0.0f;
      ctx->lock_uq = 0.0f;
      break;
  }
}

/**
 * @brief 对齐准备: 初始化对齐上下文, 设置锁定轴, 更新传感器状态, 记录初始电角。
 * 
 * @param ctx 
 * @return error_t 
 */
static error_t cal_prepare(cal_ctx_t *ctx)
{
  if ((ctx == NULL) || (ctx->m == NULL) || (ctx->rt == NULL))
  {
    return ERR_NULL;
  }

  ctx->m->config.param.theta_elec_offset_rad = 0.0f;
  ctx->dir = ctx->m->config.param.direction;
  ctx->raw_locked = 0.0f;
  ctx->offset_rad = 0.0f;

  ctx->rt->align_direction_flipped = false;
  ctx->rt->align_ok = false;
  ctx->rt->align_engaged = false;
  ctx->rt->align_raw_elec_rad = 0.0f;
  ctx->rt->align_raw_engage_rad = 0.0f;
  ctx->rt->align_polarity_test_iq = 0.0f;

  cal_setup_lock_axes(ctx);

  foc_sense_update(ctx->m);
  ctx->rt->align_theta_before_rad = ctx->m->state.theta_elec_rad;

  motor_arm(ctx->m);
  ctx->m->out.pwm_enable = 1u;
  return ERR_NONE;
}

/**
 * @brief 锁定采样: 保持电压, 采样电流, 检查分散度。
 * 
 * @param ctx 
 * @param angle_el 
 * @param hold_ms 保持时间
 * @param spread_out 分散度输出
 * @return error_t 
 */
static error_t cal_lock_sample_hold(cal_ctx_t *ctx,
                                    float angle_el,
                                    uint32_t hold_ms,
                                    float *spread_out)
{
  float spread = 0.0f;

  cal_hold_voltage(ctx, ctx->lock_ud, ctx->lock_uq, angle_el, hold_ms);
  ctx->raw_locked = cal_sample_raw_at_lock(ctx,
                                           ctx->lock_ud,
                                           ctx->lock_uq,
                                           angle_el,
                                           &spread);
  if (spread_out != NULL)
  {
    *spread_out = spread;
  }
  if (!cal_spread_ok(ctx, spread))
  {
    motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
    return ERR_ALIGIN_SPREAD_OUT;
  }
  return ERR_NONE;
}

static error_t cal_lock_sample(cal_ctx_t *ctx, float angle_el, float *spread_out)
{
  return cal_lock_sample_hold(ctx, angle_el, ctx->cfg.hold_ms, spread_out);
}

/**
 * @brief 检查转子是否吸合
 *   原理: 在转子期望电角θ_align + π处施加电压, 如果转子吸合, 则电流会上升, 否则会下降。
 * 
 * @param ctx 
 * @param raw_locked 
 * @param raw_engage_out 
 * @param spread_out 
 * @return uint8_t 
 */
static uint8_t cal_check_rotor_engaged(cal_ctx_t *ctx,
                                       float ud,
                                       float raw_locked,
                                       float *raw_engage_out,
                                       float *spread_out)
{
  const float engage_angle = angle_normalize(ctx->align_angle + _PI);
  float raw_engage;
  float spread;
  float delta;

  /* 与旧 foc_align_check_rotor_engaged 一致：反相点仅 Ud，Uq=0；π 步进需更长保持 */
  cal_hold_voltage(ctx, ud, 0.0f, engage_angle, ctx->cfg.hold_ms);
  raw_engage = cal_sample_raw_at_lock(ctx,
                                      ud,
                                      0.0f,
                                      engage_angle,
                                      &spread);

  if (raw_engage_out != NULL)
  {
    *raw_engage_out = raw_engage;
  }
  if (spread_out != NULL)
  {
    *spread_out = spread;
  }

  delta = fabsf(angle_normalize(raw_engage - raw_locked));
  if (delta < _PI - ctx->cfg.rotor_engage_tol_rad)
  {
    return 0u;
  }
  if (spread > ctx->cfg.rotor_spread_max_rad)
  {
    return 0u;
  }
  return 1u;
}

/**
 * @brief 极性校正: 校正极性, 检查极性, 计算 offset。
 *   原理: +Vq 极性探针期望 i_q > 0（相电流极性已在 current_sense 中修正）。
 * 
 * @param ctx 
 * @param iq_probe_out 
 * @return error_t 
 */
static error_t cal_polarity_correct(cal_ctx_t *ctx, float *iq_probe_out)
{
  float iq_probe;
  float spread = 0.0f;
  float ud_polarity = ctx->lock_ud;
  float uq_lock = ctx->lock_uq;
  const bool uq_lock_axis = (cal_ctx_lock_axis(ctx) == CAL_LOCK_AXIS_UQ);

  if ((ctx == NULL) || (iq_probe_out == NULL))
  {
    return ERR_NULL;
  }

  if (uq_lock_axis)
  {
    iq_probe = cal_probe_id_uq_lock(ctx,
                                    ctx->cfg.polarity_test_uq_v,
                                    uq_lock,
                                    ctx->align_angle,
                                    ctx->cfg.polarity_test_ms);
  }
  else
  {
    iq_probe = cal_probe_iq(ctx,
                            ctx->cfg.polarity_test_uq_v,
                            ud_polarity,
                            ctx->align_angle,
                            ctx->cfg.polarity_test_ms);
  }

  if (cal_polarity_probe_needs_flip(iq_probe, ctx->cfg.polarity_iq_min_a))
  {
    cal_flip_direction(ctx);

    cal_hold_voltage(ctx, ud_polarity, uq_lock, ctx->align_angle, ctx->cfg.relock_ms);
    ctx->raw_locked = cal_sample_raw_at_lock(ctx,
                                             ud_polarity,
                                             uq_lock,
                                             ctx->align_angle,
                                             &spread);
    if (!cal_spread_ok(ctx, spread))
    {
      motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
      return ERR_ALIGIN_SPREAD_OUT;
    }

    if (uq_lock_axis)
    {
      iq_probe = cal_probe_id_uq_lock(ctx,
                                      ctx->cfg.polarity_test_uq_v,
                                      uq_lock,
                                      ctx->align_angle,
                                      ctx->cfg.polarity_test_ms);
    }
    else
    {
      iq_probe = cal_probe_iq(ctx,
                              ctx->cfg.polarity_test_uq_v,
                              ud_polarity,
                              ctx->align_angle,
                              ctx->cfg.polarity_test_ms);
    }
  }

  *iq_probe_out = iq_probe;
  return ERR_NONE;
}

static void cal_commit_offset(cal_ctx_t *ctx)
{
  if ((ctx == NULL) || (ctx->m == NULL) || (ctx->rt == NULL))
  {
    return;
  }

  ctx->m->config.param.theta_elec_offset_rad = ctx->offset_rad;

  if ((ctx->m->config.param.direction != DIR_CW) && (ctx->m->config.param.direction != DIR_CCW))
  {
    ctx->m->config.param.direction = DIR_CW;
  }

  foc_sense_update(ctx->m);
  ctx->rt->align_theta_after_rad = ctx->m->state.theta_elec_rad;
  ctx->rt->open_loop_elec_angle_rad = ctx->m->state.theta_elec_rad;
}

static void cal_finish_hw(cal_ctx_t *ctx)
{
  if ((ctx == NULL) || (ctx->m == NULL))
  {
    return;
  }

  foc_voltage(ctx->m, 0.0f, 0.0f, ctx->align_angle);
  delay_ms(50u);
  (void)foc_pid_clear(&ctx->m->id_pid);
  (void)foc_pid_clear(&ctx->m->iq_pid);
}

/**
 * @brief Ud 单点对齐（原 foc_align_electrical 流程）。
 *
 * 在 θ_open 下施加 Ud 锁定转子 d 轴，采样编码器 raw 电角 raw_locked，
 * 写 offset = normalize(raw_locked − θ_open)，使 theta_elec = θ_open。
 */
static error_t cal_method_ud(cal_ctx_t *ctx)
{
  error_t ret = ERR_NONE;
  float spread = 0.0f;
  float raw_engage = 0.0f;
  float iq_probe = 0.0f;
  const float ud = ctx->cfg.voltage_d_v;
  const float uq_assist = ctx->cfg.voltage_q_v;

  if ((ctx == NULL) || (ctx->m == NULL) || (ctx->rt == NULL))
  {
    return ERR_NULL;
  }

  cal_hold_voltage(ctx, ud, uq_assist, ctx->align_angle, ctx->cfg.hold_ms);
  ctx->raw_locked = cal_sample_raw_at_lock(ctx,
                                           ud,
                                           uq_assist,
                                           ctx->align_angle,
                                           &spread);
  if (!cal_spread_ok(ctx, spread))
  {
    motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
    return ERR_ALIGIN_SPREAD_OUT;
  }
  ctx->rt->align_raw_elec_rad = ctx->raw_locked;
#if 1
  /** 极性校正: 检查极性, 计算 offset。 
   *  
   *  目的是确定：编码器电角度方向和电机磁场旋转方向是否一致
   */
  iq_probe = cal_probe_iq(ctx,
                          ctx->cfg.polarity_test_uq_v,
                          /*ud,*/ 0.0f,
                          ctx->align_angle,
                          ctx->cfg.polarity_test_ms);
  if (cal_polarity_probe_needs_flip(iq_probe, ctx->cfg.polarity_iq_min_a))
  {
    cal_flip_direction(ctx);

    cal_hold_voltage(ctx, ud, 0.0f, ctx->align_angle, ctx->cfg.relock_ms);
    ctx->raw_locked = cal_sample_raw_at_lock(ctx,
                                             ud,
                                             0.0f,
                                             ctx->align_angle,
                                             &spread);
    if (!cal_spread_ok(ctx, spread))
    {
      motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
      return ERR_ALIGIN_SPREAD_OUT;
    }
    ctx->rt->align_raw_elec_rad = ctx->raw_locked;
    iq_probe = cal_probe_iq(ctx,
                            ctx->cfg.polarity_test_uq_v,
                            /*ud,*/ 0.0f,
                            ctx->align_angle,
                            ctx->cfg.polarity_test_ms);
  }

  /* 重新锁定转子 */
  cal_hold_voltage(ctx, ud, 0.0f, ctx->align_angle, ctx->cfg.relock_ms);
  ctx->raw_locked = cal_sample_raw_at_lock(ctx,
                                           ud,
                                           0.0f,
                                           ctx->align_angle,
                                           &spread);
  if (!cal_spread_ok(ctx, spread))
  {
    motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
    return ERR_ALIGIN_SPREAD_OUT;
  }
  ctx->rt->align_raw_elec_rad = ctx->raw_locked;

  ctx->rt->align_engaged = cal_check_rotor_engaged(ctx,
                                                   ud,
                                                   ctx->raw_locked,
                                                   &raw_engage,
                                                   &spread);
  ctx->rt->align_raw_engage_rad = raw_engage;
  /* offset 仅由 θ_open 锁定采样决定；吸合验证为诊断项，不阻断 Ud 对齐 */

  ctx->rt->align_polarity_test_iq = iq_probe;
  if (cal_polarity_probe_aligned(iq_probe, ctx->cfg.polarity_iq_min_a))
  {
    ctx->rt->align_ok = true;
  }
  else
  {
    motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
    ret = ERR_ALIGIN_FAILED;
    return ret;
  }
#else
  ctx->rt->align_ok = true;
#endif
  ctx->offset_rad = angle_normalize(ctx->raw_locked - ctx->align_angle);
  return ERR_NONE;
}

/**
 * @brief Uq 单点对齐（dex-con）：Uq 锁定 → 等待 → offset = raw。
 * 与 Ud 的 offset = raw − θ_open 是两套不同零位定义，结果不同是预期行为。
 */
static error_t cal_method_uq_dex(cal_ctx_t *ctx)
{
  float raw;

  if ((ctx == NULL) || (ctx->m == NULL) || (ctx->rt == NULL))
  {
    return ERR_NULL;
  }

  ctx->align_angle = ctx->cfg.align_angle_rad;
  ctx->lock_ud = 0.0f;
  ctx->lock_uq = ctx->cfg.voltage_q_v;

  cal_hold_voltage(ctx, ctx->lock_ud, ctx->lock_uq, ctx->align_angle, ALIGN_HOLD_MS_DEX);
  foc_sense_update(ctx->m);
  raw = cal_encoder_raw_elec_rad(ctx->m);

  ctx->raw_locked = raw;
  ctx->offset_rad = cal_offset_uq_dex(raw);
  ctx->rt->align_raw_elec_rad = raw;
  ctx->rt->align_engaged = true;
  ctx->rt->align_ok = true;
  return ERR_NONE;
}

/** @brief 双点强拖对齐（静摩擦对消） */
static error_t cal_method_dual_point(cal_ctx_t *ctx)
{
  error_t ret;
  float spread = 0.0f;
  float raw_second = 0.0f;
  float iq_probe = 0.0f;
  const float angle_second = angle_normalize(ctx->align_angle + ctx->cfg.dual_point_delta_rad);

  ret = cal_lock_sample(ctx, ctx->align_angle, &spread);
  if (ret != ERR_NONE)
  {
    return ret;
  }

  ret = cal_lock_sample(ctx, angle_second, &spread);
  if (ret != ERR_NONE)
  {
    return ret;
  }
  raw_second = ctx->raw_locked;

  ret = cal_lock_sample(ctx, ctx->align_angle, &spread);
  if (ret != ERR_NONE)
  {
    return ret;
  }

  ctx->offset_rad = cal_offset_dual_friction_avg(ctx,
                                               ctx->raw_locked,
                                               ctx->align_angle,
                                               raw_second,
                                               angle_second);
  ctx->raw_locked =
      angle_normalize(cal_rotor_d_axis_ref_rad(ctx, ctx->align_angle) + ctx->offset_rad);
  ctx->rt->align_raw_elec_rad = ctx->raw_locked;
  ctx->rt->align_engaged = true;

  ret = cal_polarity_correct(ctx, &iq_probe);
  if (ret != ERR_NONE)
  {
    return ret;
  }
  ctx->rt->align_polarity_test_iq = iq_probe;
  if (!cal_polarity_probe_aligned(iq_probe, ctx->cfg.polarity_iq_min_a))
  {
    motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
    return ERR_ALIGIN_FAILED;
  }

  ctx->rt->align_ok = true;
  return ERR_NONE;
}

static float cal_hfi_probe_metric(cal_ctx_t *ctx, float angle_base)
{
  float omega = _2PI * ctx->cfg.hfi_freq_hz;
  uint32_t period_ms = (uint32_t)(1000.0f / ctx->cfg.hfi_freq_hz + 0.5f);
  uint32_t total_ms = period_ms * (uint32_t)ctx->cfg.hfi_cycles;
  uint32_t elapsed = 0u;
  float sum_metric = 0.0f;
  uint32_t n = 0u;

  if ((period_ms == 0u) || (ctx->cfg.hfi_scan_steps == 0u))
  {
    return 0.0f;
  }

  while (elapsed < total_ms)
  {
    float t_s = (float)elapsed * 0.001f;
    float carrier = sinf(omega * t_s);
    float angle_el = angle_normalize(angle_base + omega * t_s);

    foc_voltage(ctx->m, 0.0f, ctx->cfg.hfi_voltage_v * carrier, angle_el);
    foc_sense_update(ctx->m);
    sum_metric += fabsf(ctx->m->state.i_d) + fabsf(ctx->m->state.i_q);
    n++;
    delay_ms(ctx->cfg.sample_interval_ms);
    elapsed += ctx->cfg.sample_interval_ms;
  }

  return (n > 0u) ? (sum_metric / (float)n) : 0.0f;
}

static error_t cal_method_hfi(cal_ctx_t *ctx)
{
  float best_metric = -1.0f;
  float best_angle = 0.0f;
  float spread = 0.0f;
  float iq_probe = 0.0f;
  error_t ret;
  uint8_t steps = ctx->cfg.hfi_scan_steps;

  if (steps < 8u)
  {
    steps = 8u;
  }

  for (uint8_t i = 0u; i < steps; i++)
  {
    float probe = _2PI * (float)i / (float)steps;
    float metric = cal_hfi_probe_metric(ctx, probe);

    if (metric > best_metric)
    {
      best_metric = metric;
      best_angle = probe;
    }
  }

  if (best_metric <= 0.0f)
  {
    motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
    return ERR_ALIGIN_FAILED;
  }

  ctx->align_angle = best_angle;
  foc_voltage(ctx->m, 0.0f, 0.0f, ctx->align_angle);
  delay_ms(ctx->cfg.relock_ms);
  foc_sense_update(ctx->m);
  ctx->raw_locked = cal_encoder_raw_elec_rad(ctx->m);
  ctx->rt->align_raw_elec_rad = ctx->raw_locked;
  ctx->rt->align_engaged = true;

  ret = cal_polarity_correct(ctx, &iq_probe);
  if (ret != ERR_NONE)
  {
    return ret;
  }
  ctx->rt->align_polarity_test_iq = iq_probe;

  cal_hold_voltage(ctx, ctx->cfg.voltage_d_v, 0.0f, ctx->align_angle, ctx->cfg.relock_ms);
  ctx->raw_locked = cal_sample_raw_at_lock(ctx,
                                           ctx->cfg.voltage_d_v,
                                           0.0f,
                                           ctx->align_angle,
                                           &spread);
  if (!cal_spread_ok(ctx, spread))
  {
    motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
    return ERR_ALIGIN_SPREAD_OUT;
  }
  ctx->rt->align_raw_elec_rad = ctx->raw_locked;

  if (!cal_polarity_probe_aligned(iq_probe, ctx->cfg.polarity_iq_min_a))
  {
    motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
    return ERR_ALIGIN_FAILED;
  }

  ctx->offset_rad = cal_offset_from_sample(ctx, ctx->raw_locked, ctx->align_angle);
  ctx->rt->align_ok = true;
  return ERR_NONE;
}

static error_t cal_elec_align_run(cal_ctx_t *ctx)
{
  error_t ret;

  if ((ctx == NULL) || (ctx->m == NULL) || (ctx->rt == NULL))
  {
    return ERR_NULL;
  }

  ret = cal_prepare(ctx);
  if (ret != ERR_NONE)
  {
    goto exit;
  }

  switch (ctx->method)
  {
    case CAL_ELEC_ALIGN_UD:
      ret = cal_method_ud(ctx);
      break;

    case CAL_ELEC_ALIGN_UQ:
      ret = cal_method_uq_dex(ctx);
      break;

    case CAL_ELEC_ALIGN_DUAL_POINT:
    case CAL_ELEC_ALIGN_DUAL_POINT_UQ:
      ret = cal_method_dual_point(ctx);
      break;

    case CAL_ELEC_ALIGN_HFI:
      ret = cal_method_hfi(ctx);
      break;

    default:
      ret = ERR_PARAM;
      break;
  }

  if ((ret == ERR_NONE) && ctx->rt->align_ok)
  {
    cal_commit_offset(ctx);
    ctx->rt->electrical_align_done = true;
  }
  else
  {
    ctx->rt->electrical_align_done = false;
    if ((ret == ERR_NONE) && !ctx->rt->align_ok)
    {
      motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
      ret = ERR_ALIGIN_FAILED;
    }
  }

  cal_finish_hw(ctx);

exit:
  motor_disarm(ctx->m);
  return ret;
}

error_t cal_elec_align_ex(motor_handle_t *m,
                          cal_elec_align_method_t method,
                          const cal_elec_align_cfg_t *cfg)
{
  cal_ctx_t ctx;
  cal_elec_align_cfg_t local_cfg;

  if (m == NULL) return ERR_NULL;

  memset(&ctx, 0, sizeof(ctx));
  ctx.m = m;
  ctx.rt = motor_runtime(m);

  if (ctx.rt == NULL) return ERR_NULL;

  if (cfg != NULL)
  {
    ctx.cfg = *cfg;
  }
  else
  {
    cal_elec_align_cfg_default(&local_cfg);
    ctx.cfg = local_cfg;
  }
  ctx.method = method;

  return cal_elec_align_run(&ctx);
}

/** 
 * @brief 电角度对齐: 使用默认方法对齐电角度。
 * 
 *   当前只在初始化时自动对齐时用.
 * 
 * @param m 电机句柄
 * @return error_t 
 */
error_t cal_elec_align(motor_handle_t *m)
{
  return cal_elec_align_ex(m, CAL_ELEC_ALIGN_METHOD_DEFAULT, NULL);
}
