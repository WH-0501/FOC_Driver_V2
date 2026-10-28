#include "foc_calibration.h"
#include "foc.h"
#include "board.h"
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
  /** 对齐角度（默认 ALIGN_ELECTRICAL_ANGLE_RAD: 3π/2） */
  float align_angle;
  /** 锁定电压（默认 ALIGN_ELECTRICAL_VOLTAGE_D_V） */
  float lock_ud;
  /** 锁定电压（默认 ALIGN_ELECTRICAL_VOLTAGE_Q_V） */
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
  return m->state.shaft.angle_rad * pole_pairs; // 电机轴单圈机械角度 × pole_pairs = 电机单圈电角度
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

  if (CURRENT_SENSE_PHASE_POLARITY < 0.0f)
  {
    return (iq_probe > 0.0f);
  }

  return (iq_probe < 0.0f);
}

static bool cal_polarity_probe_aligned(float iq_probe, float iq_min_a)
{
  if (fabsf(iq_probe) < iq_min_a)
  {
    return true;
  }

  if (CURRENT_SENSE_PHASE_POLARITY < 0.0f)
  {
    return (iq_probe < 0.0f);
  }

  return (iq_probe > 0.0f);
}



static float cal_expected_theta_elec_rad(const cal_ctx_t *ctx)
{
  if (ctx == NULL)
  {
    return 0.0f;
  }

  switch (ctx->method)
  {
    case CAL_ELEC_ALIGN_UQ:
    case CAL_ELEC_ALIGN_DUAL_POINT_UQ:
      return 0.0f;

    case CAL_ELEC_ALIGN_UD:
    case CAL_ELEC_ALIGN_DUAL_POINT:
    case CAL_ELEC_ALIGN_HFI:
    default:
      return ctx->align_angle;
  }
}

static bool cal_theta_matches_expected(const cal_ctx_t *ctx, float tol_rad)
{
  float expect;
  float err;

  if ((ctx == NULL) || (ctx->m == NULL))
  {
    return false;
  }

  foc_sense_update(ctx->m);
  expect = cal_expected_theta_elec_rad(ctx);
  err = fabsf(angle_normalize(ctx->m->state.theta_elec_rad - expect + _PI) - _PI);
  return err <= tol_rad;
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

static float cal_sample_raw_at_lock(cal_ctx_t *ctx, float ud, float uq,
                                    float angle_el, float *spread_out)
{
  uint8_t i;
  float first = 0.0f, sum = 0.0f, lo = 0.0f, hi = 0.0f;

  if (spread_out != NULL) *spread_out = 1e9f;
  if (ctx == NULL || ctx->m == NULL || ctx->cfg.sample_count == 0u) return 0.0f;

  for (i = 0; i < ctx->cfg.sample_count; ++i)
  {
    float raw, delta;
    foc_voltage(ctx->m, uq, ud, angle_el);
    foc_sense_update(ctx->m);
    raw = angle_normalize(cal_encoder_raw_elec_rad(ctx->m)); // 电角度归一化
    /** 记录初次采样值,后续多次采样计算相对该初值的 spread, 并计算平均差值修正采样值 */
    if (i == 0u) first = raw;
    delta = angle_normalize(raw - first + _PI) - _PI;
    sum += delta;
    if (delta < lo) lo = delta;
    if (delta > hi) hi = delta;
    delay_ms(ctx->cfg.sample_interval_ms);
  }
  if (spread_out != NULL) *spread_out = hi - lo;
  return angle_normalize(first + sum / (float)ctx->cfg.sample_count);
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
  return angle_normalize(off0 + 0.5f * (angle_normalize(off1 - off0 + _PI) - _PI));
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
  
  // 保持电压，角度为angle_el，持续hold_ms时间
  cal_hold_voltage(ctx, ctx->lock_ud, ctx->lock_uq, angle_el, hold_ms);
  // 采样编码器raw电角
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

/**
 * @brief 锁定采样。在角度为angle_el下，保持电压，采样编码器raw电角，检查分散度。
 * 
 * @param ctx 
 * @param angle_el 
 * @param spread_out 
 * @return error_t 
 */
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
 * @brief 扫角。
 *   1. 从 start 到 end，分 steps 步，每步 cal_hold_voltage sample_interval_ms 时间。
 *   2. 终点在 hold_ms 稳定。
 *   3. 扫角过程中，电机轴机械角度变化量 = 当前电机轴机械角度 - 上次电机轴机械角度 + π。
 * 
 * Sweep one electrical revolution each way. Mechanical increments are
 * unwrapped before multiplying by pole pairs, including the one-pole case.
 * Direction is measured from motion, never inferred from current polarity
 */
static float cal_ud_sweep(cal_ctx_t *ctx, float start, float end)
{
  const uint32_t steps = 360u;
  uint32_t i;
  float prev, travel = 0.0f;
  foc_sense_update(ctx->m);
  prev = ctx->m->state.shaft.angle_rad;
  for (i = 1u; i <= steps; ++i)
  {
    float angle = start + (end - start) * (float)i / (float)steps;
    float now;
    cal_hold_voltage(ctx, ctx->cfg.voltage_d_v, 0.0f,
                     angle_normalize(angle), ctx->cfg.sample_interval_ms);
    foc_sense_update(ctx->m);
    now = ctx->m->state.shaft.angle_rad; // 电机轴单圈机械角度
    /** 等价于把相邻两帧机械角差折到 (−π, π]，再 累加，得到 不绕 ±π 断层的总转角（unwrap 累加） */
    travel += angle_normalize(now - prev + _PI) - _PI; // 每一步的增量累积
    prev = now;
  }
  cal_hold_voltage(ctx, ctx->cfg.voltage_d_v, 0.0f,
                   angle_normalize(end), ctx->cfg.hold_ms);
  foc_sense_update(ctx->m);
  /** 最后一步的增量累积，避免终点慢爬/微调没进循环最后一步 */
  travel += angle_normalize(ctx->m->state.shaft.angle_rad - prev + _PI) - _PI;
  return travel;
}

static error_t cal_method_ud(cal_ctx_t *ctx)
{
  float forward, reverse, expected, spread, base, probe;
  error_t ret;

  ctx->lock_ud = ctx->cfg.voltage_d_v;
  ctx->lock_uq = 0.0f;

  ret = cal_lock_sample(ctx, ctx->align_angle, &spread);
  if (ret != ERR_NONE) return ret;
  /** 正向扫角。从对齐角度开始，扫到对齐角度 + 2π。分 360 步，每步 cal_hold_voltage 5ms */
  forward = cal_ud_sweep(ctx, ctx->align_angle, ctx->align_angle + _2PI);
  /** 反向扫角。从对齐角度 + 2π开始，扫到对齐角度 */
  reverse = cal_ud_sweep(ctx, ctx->align_angle + _2PI, ctx->align_angle);
  /** 期望值。2π / 极对数 = 电机单圈电角度 */
  expected = _2PI / (float)ctx->m->config.param.pole_pairs;
  /**
   * Each leg must follow the field with the configured pole count (15%). 
   * forward × reverse < 0：往返方向相反；
   * |forward|、|reverse| ≈ 2π / pole_pairs（±15%）：与配置的 极对数 一致，且没有严重打滑/跟不全
   */
  if (!(forward * reverse < 0.0f) ||
      fabsf(fabsf(forward) - expected) > 0.15f * expected ||
      fabsf(fabsf(reverse) - expected) > 0.15f * expected)
    return ERR_ALIGIN_FAILED;
  /** forward > 0：场从 θ_open 增到 θ_open+2π 时，编码器认为转子沿 正方向 转了约 expected rad, 否则反向 */
  if (forward < 0.0f) cal_flip_direction(ctx); // 正向扫角小于0，则翻转方向
  
  /**
   * 再次正反扫角 π/2。作用：
   * 1. 跟场（egage）：
   *    只在 θ_open 锁一次只能证明「这一点吸合」；把场 +90° 电角 后，若转子 真的跟场，编码器 raw 电角也应 大约 +90°。
   *    若 180° 级错、没跟转、只弹性形变，probe − base 不会接近 π/2 → 对齐失败。
   *    non-antipodal: 用 90° 而不是 180° 作为检查点，避免和对称/对极点混淆时只靠 π 步进不够敏感。
   * 2. 可重复/无滑移：
   *    再扫回 θ_open 时，若转子 真的跟场，raw 应回到 base。
   *    若跟场过程中 打滑、失步、或 spread 合格但平均 raw 漂移，回零对不上 → 失败。
   *    避免「单点 raw 看起来稳，但一动就错、算出的 offset 不可信」
   * 3. 若不使用 sweep, 直接在 θ_open + π/2 锁一次，转子可能还在旧位置，probe没有意义。
   */
  ret = cal_lock_sample(ctx, ctx->align_angle, &spread);
  if (ret != ERR_NONE) return ret;
  base = ctx->raw_locked;
  /* Verify a non-antipodal point, then return before computing the offset. */
  (void)cal_ud_sweep(ctx, ctx->align_angle, ctx->align_angle + 0.5f * _PI);
  ret = cal_lock_sample(ctx, angle_normalize(ctx->align_angle + 0.5f * _PI), &spread);
  if (ret != ERR_NONE) return ret;
  probe = ctx->raw_locked;
  if (fabsf(angle_normalize(probe - base - 0.5f * _PI + _PI) - _PI) >
      ctx->cfg.rotor_engage_tol_rad) return ERR_ALIGIN_FAILED;
  (void)cal_ud_sweep(ctx, ctx->align_angle + 0.5f * _PI, ctx->align_angle);
  ret = cal_lock_sample(ctx, ctx->align_angle, &spread);
  if (ret != ERR_NONE) return ret;
  if (fabsf(angle_normalize(ctx->raw_locked - base + _PI) - _PI) >
      ctx->cfg.rotor_engage_tol_rad) return ERR_ALIGIN_FAILED;

  ctx->offset_rad = angle_normalize(ctx->raw_locked - ctx->align_angle);
  ctx->rt->align_raw_elec_rad = ctx->raw_locked;
  ctx->rt->align_raw_engage_rad = probe;
  ctx->rt->align_engaged = true;
  /* No current-polarity probe is performed by the UD method. */
  ctx->rt->align_polarity_test_iq = 0.0f;
  ctx->rt->align_ok = true;
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
  float saved_offset;
  int8_t saved_direction;

  if ((ctx == NULL) || (ctx->m == NULL) || (ctx->rt == NULL))
  {
    return ERR_NULL;
  }

  saved_offset = ctx->m->config.param.theta_elec_offset_rad;
  saved_direction = ctx->m->config.param.direction;
  if (ctx->m->config.param.direction != DIR_CW &&
      ctx->m->config.param.direction != DIR_CCW)
    ctx->m->config.param.direction = DIR_CW;
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
    if (!cal_theta_matches_expected(ctx, 0.5f))
    {
      ctx->rt->align_ok = false;
      ctx->rt->electrical_align_done = false;
      motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
      ret = ERR_ALIGIN_FAILED;
    }
    else
    {
      ctx->rt->electrical_align_done = true;
    }
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
  if (ret != ERR_NONE)
  {
    ctx->m->config.param.theta_elec_offset_rad = saved_offset;
    ctx->m->config.param.direction = saved_direction;
    ctx->rt->align_ok = false;
    ctx->rt->electrical_align_done = false;
    motor_fault_raise(ctx->m, FAULT_ALIGN_FAILED);
    foc_sense_update(ctx->m);
    ctx->rt->open_loop_elec_angle_rad = ctx->m->state.theta_elec_rad;
  }
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

  if (ctx.cfg.sample_count == 0u || ctx.cfg.sample_interval_ms == 0u ||
      ctx.cfg.sample_interval_ms > 100u || ctx.cfg.hold_ms == 0u ||
      m->config.param.pole_pairs == 0u ||
      !(ctx.cfg.rotor_spread_max_rad > 0.0f) ||
      !(ctx.cfg.rotor_engage_tol_rad > 0.0f) ||
      !(ctx.cfg.voltage_d_v > 0.0f)) return ERR_PARAM;
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

/* Read-only verification of the existing calibration, not another alignment.
 * Bounded foreground operation; callback failure and hardware faults stop PWM.
 * All angles sent to the observer use the existing direction/offset unchanged. */
typedef struct {
  motor_handle_t *m;
  cal_verify_report_fn report;
  void *user;
  cal_verify_sample_t sample;
  float prev_mech;
  float max_error;
  uint16_t clock_us;
  uint32_t elapsed_us;
  uint32_t last_report_ms;
  bool baseline_set;
  bool failed;
} cal_verify_ctx_t;

static error_t cal_verify_tick(cal_verify_ctx_t *v, float target,
                               uint8_t phase, bool settled)
{
  float measured, mech;
  uint16_t now, wait_start;
  if (motor_fault_is_latched(v->m)) return ERR_FAULT_LATCHED;
  foc_voltage(v->m, 0.0f, ALIGN_ELECTRICAL_VOLTAGE_D_V, angle_normalize(target));
  wait_start = (uint16_t)board_time_us();
  delay_ms(5u);
  /* Independent timer detects a broken DWT delay before the field can sweep. */
  if ((uint16_t)((uint16_t)board_time_us() - wait_start) < 4000u)
    return ERR_TIMEOUT;
  foc_sense_poll_vbus(v->m);
  foc_sense_update(v->m);
  motor_fault_protect(v->m);
  if (motor_fault_is_latched(v->m) || !motor_is_armed(v->m)) return ERR_FAULT_LATCHED;
  now = (uint16_t)board_time_us();
  v->elapsed_us += (uint16_t)(now - v->clock_us);
  v->clock_us = now;
  mech = v->m->state.shaft.angle_rad;
  measured = v->m->state.theta_elec_rad;
  if (!(mech >= 0.0f && mech < _2PI) ||
      !(measured >= 0.0f && measured < _2PI)) return ERR_SENSOR;
  if (v->baseline_set)
    v->sample.mechanical_rad += angle_normalize(mech - v->prev_mech + _PI) - _PI;
  v->prev_mech = mech;
  if (phase == 0u && settled) {
    v->baseline_set = true;
    v->sample.mechanical_rad = 0.0f;
  }
  v->sample.elapsed_ms = v->elapsed_us / 1000u;
  v->sample.phase = phase;
  v->sample.settled = settled;
  v->sample.target_rad = angle_normalize(target);
  v->sample.measured_rad = measured;
  v->sample.error_rad = angle_normalize(measured - target + _PI) - _PI;
  v->sample.expected_mechanical_rad = (target - ALIGN_ELECTRICAL_ANGLE_RAD) /
                                      (float)v->m->config.param.pole_pairs;
  if (settled) {
    float error = fabsf(v->sample.error_rad);
    if (error > v->max_error) v->max_error = error;
    if (error > ALIGN_ROTOR_ENGAGE_TOL_RAD) v->failed = true;
    /* Independently check accumulated mechanical motion; catches wrong pole
     * count even when wrapped electrical angles happen to agree at endpoints. */
    if (fabsf(v->sample.mechanical_rad - v->sample.expected_mechanical_rad) >
        0.15f * _2PI / (float)v->m->config.param.pole_pairs) v->failed = true;
  }
  if (settled || v->sample.elapsed_ms - v->last_report_ms >= 50u) {
    if (!v->report(&v->sample, v->user)) return ERR_COMM;
    v->last_report_ms = v->sample.elapsed_ms;
    v->sample.sequence++;
  }
  return ERR_NONE;
}

error_t cal_verify_electrical(motor_handle_t *m, cal_verify_report_fn report,
                             void *user, uint16_t *samples, float *max_error)
{
  cal_verify_ctx_t v;
  error_t ret = ERR_NONE;
  unsigned i, leg, hold;
  float from, target;
  if (samples != NULL) *samples = 0u;
  if (max_error != NULL) *max_error = 0.0f;
  if (m == NULL || report == NULL) return ERR_NULL;
  if (motor_fault_is_latched(m)) return ERR_FAULT_LATCHED;
  if (motor_is_armed(m) || m->fsm != STATE_IDLE) return ERR_STATE;
  if (!m->current_offset_cal_done) return ERR_OFFSET_CAL;
  if (m->config.param.pole_pairs == 0u ||
      (m->config.param.direction != DIR_CW && m->config.param.direction != DIR_CCW)) return ERR_PARAM;
  memset(&v, 0, sizeof(v));
  v.m = m; v.report = report; v.user = user;
  v.clock_us = (uint16_t)board_time_us();
  motor_arm(m);
  if (!motor_is_armed(m)) {ret = ERR_STATE; goto cleanup;}
  for (hold = 0u; hold < 140u; ++hold) {
    ret = cal_verify_tick(&v, ALIGN_ELECTRICAL_ANGLE_RAD, 0u, hold == 139u);
    if (ret != ERR_NONE) goto cleanup;
  }
  from = ALIGN_ELECTRICAL_ANGLE_RAD;
  for (leg = 0u; leg < 8u; ++leg) {
    uint8_t phase = (leg < 4u) ? 1u : 2u;
    target = from + ((leg < 4u) ? 0.5f * _PI : -0.5f * _PI);
    for (i = 1u; i <= 90u; ++i) {
      ret = cal_verify_tick(&v, from + (target - from) * (float)i / 90.0f, phase, false);
      if (ret != ERR_NONE) goto cleanup;
    }
    for (hold = 0u; hold < 70u; ++hold) {
      ret = cal_verify_tick(&v, target, phase, hold == 69u);
      if (ret != ERR_NONE) goto cleanup;
    }
    from = target;
  }
  if (v.failed) ret = ERR_ALIGIN_FAILED;
cleanup:
  /* Disarm before emitting the final protocol reply. Parameters are untouched. */
  motor_disarm(m);
  (void)foc_pid_clear(&m->id_pid);
  (void)foc_pid_clear(&m->iq_pid);
  if (samples != NULL) *samples = v.sample.sequence;
  if (max_error != NULL) *max_error = v.max_error;
  return ret;
}
