#ifndef __FOC_CALIBRATION_H__
#define __FOC_CALIBRATION_H__

#include "datatypes.h"
#include "error_types.h"

/**
 * @brief 电角度零位对齐方案。
 *
 * - UD：d 轴锁定；offset = raw − θ_open；对齐后 theta_elec ≈ θ_open
 * - UQ：q 轴锁定（dex-con）；offset = raw；对齐后 theta_elec ≈ 0（与 UD 公式不同，勿混比）
 * - DUAL_POINT：Ud 双点静摩擦对消，offset 公式同 UD
 * - DUAL_POINT_UQ：Uq 双点，offset = raw − (θ_open + uq_d_axis_offset)
 * - HFI：扫角后按 UD 公式写 offset
 */
typedef enum
{
  CAL_ELEC_ALIGN_UD = 0,
  CAL_ELEC_ALIGN_UQ,
  CAL_ELEC_ALIGN_DUAL_POINT,
  CAL_ELEC_ALIGN_DUAL_POINT_UQ,
  CAL_ELEC_ALIGN_HFI,
} cal_elec_align_method_t;

#ifndef CAL_ELEC_ALIGN_METHOD_DEFAULT
#define CAL_ELEC_ALIGN_METHOD_DEFAULT CAL_ELEC_ALIGN_UD
#endif

/** 对齐流程参数（NULL 时使用 cal_elec_align_cfg_default） */
typedef struct
{
  /** 对齐角度（默认 ALIGN_ELECTRICAL_ANGLE_RAD） */
  float align_angle_rad;
  /** Ud 锁定电压（UD / DUAL_POINT）；Uq 锁定见 voltage_q_v（默认 ALIGN_ELECTRICAL_VOLTAGE_D_V） */
  float voltage_d_v;
  /** Uq 锁定电压（UQ / DUAL_POINT_UQ）（默认 ALIGN_ELECTRICAL_VOLTAGE_Q_V） */
  float voltage_q_v;
  /** 锁定时间（默认 ALIGN_HOLD_MS） */
  uint32_t hold_ms;
  /** 采样点数（默认 ALIGN_SAMPLE_COUNT） */
  uint8_t sample_count;
  /** 采样间隔（默认 ALIGN_SAMPLE_INTERVAL_MS） */
  uint32_t sample_interval_ms;
  /** 重新锁定时间（默认 ALIGN_RELOCK_MS） */
  uint32_t relock_ms;
  /** 极性测试电压（默认 ALIGN_POLARITY_TEST_UQ_V） */
  float polarity_test_uq_v;
  /** 极性测试时间（默认 ALIGN_POLARITY_TEST_MS） */
  uint16_t polarity_test_ms;
  /** 极性测试电流最小值（默认 ALIGN_POLARITY_IQ_MIN_A） */
  float polarity_iq_min_a;
  /** 转子锁定容差（默认 ALIGN_ROTOR_ENGAGE_TOL_RAD） */
  float rotor_engage_tol_rad;
  /** 转子分散最大值（默认 ALIGN_ROTOR_SPREAD_MAX_RAD） */
  float rotor_spread_max_rad;
  /** DUAL_POINT：第二点相对第一点的电角偏移，默认 π */
  float dual_point_delta_rad;
  /**
   * UQ 锁定：转子 d 轴期望电角 = θ_open + uq_d_axis_offset_rad。
   * 默认 −π/2（正 Uq 拖拽时 d 轴落后 q 轴 90°）。
   */
  float uq_d_axis_offset_rad;
  /** HFI：注入幅值 [V]、频率 [Hz]、每扫角点累积周期数、扫角步数 */
  float hfi_voltage_v;
  /** HFI：频率（默认 ALIGN_HFI_FREQ_HZ） */
  float hfi_freq_hz;
  /** HFI：每扫角点累积周期数（默认 ALIGN_HFI_CYCLES） */
  uint8_t hfi_cycles;
  /** HFI：扫角步数（默认 ALIGN_HFI_SCAN_STEPS） */
  uint8_t hfi_scan_steps;
} cal_elec_align_cfg_t;

void cal_elec_align_cfg_default(cal_elec_align_cfg_t *cfg);

/**
 * @brief 电角度零位对齐（默认方案 CAL_ELEC_ALIGN_METHOD_DEFAULT）。
 * @retval 成功 ERR_NONE；失败见 ERR_ALIGIN_* 与 FAULT_ALIGN_FAILED
 */
error_t cal_elec_align(motor_handle_t *m);

/** @brief 指定方案与参数（cfg 为 NULL 时使用默认参数） */
error_t cal_elec_align_ex(motor_handle_t *m,
                          cal_elec_align_method_t method,
                          const cal_elec_align_cfg_t *cfg);

#endif /* __FOC_CALIBRATION_H__ */
