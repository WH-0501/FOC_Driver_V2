#ifndef __MC_INTERFACE_H__
#define __MC_INTERFACE_H__

#include "datatypes.h"
#include "motor_axis.h"
#include "motor_config.h"

/**
 * mc_interface 统一返回值：MC_OK(0) 表示成功。
 *
 * mc_init 失败返回负值（MC_INIT_ERR_*）；
 * mc_enable / mc_start_calibration / mc_try_recover_from_fault 失败返回正值（MC_ERR_*）。
 */
typedef enum
{
  MC_OK = 0,

  /* --- mc_init 错误（负值） --- */
  MC_INIT_ERR_GATE_HW = -1,       /**< board_get_gate_hw 失败 */
  MC_INIT_ERR_GATE_DRV = -2,      /**< gate_driver_init 失败（芯片侧） */
  MC_INIT_ERR_GATE_PARAM = -3,    /**< gate_driver_init 参数非法 */
  MC_INIT_ERR_GATE_NOT_IMPL = -4, /**< 预驱型号未实现 */
  MC_INIT_ERR_CURRENT_LOOP = -5,  /**< 电流环 ADC IRQ 启动失败 */
  MC_INIT_ERR_AXIS = -6,          /**< 轴实例无效（MOTOR_AXIS_COUNT 与表不一致） */
  MC_INIT_ERR_CONFIG = -7,        /**< 系统配置无效 */

  /* --- 轴操作错误（正值） --- */
  MC_ERR_AXIS_INVALID = 1,   /**< 轴号无效 */
  MC_ERR_FAULT = 2,          /**< 故障锁存或 STATE_FAULT */
  MC_ERR_OFFSET_CAL = 3,     /**< 电流零漂未完成 */
  MC_ERR_CTRL_MODE = 4,      /**< 控制模式不需要 PWM */
  MC_ERR_STATE = 5,          /**< FSM 状态不允许该操作 */
  MC_ERR_CALIBRATING = 6,    /**< 正在标定中 */
  MC_ERR_CAL_TYPE = 7,       /**< 标定类型无效 */
  MC_ERR_NO_FAULT = 8,       /**< 无锁存故障（恢复失败） */
  MC_ERR_AXIS_INACTIVE = 9,  /**< 占位轴，硬件未启用 */
} mc_error_t;

/** @deprecated 使用 MC_OK */
#define MC_INIT_OK MC_OK
/** @deprecated 使用 MC_INIT_ERR_GATE_HW 或 MC_INIT_ERR_GATE_DRV */
#define MC_INIT_ERR_GATE MC_INIT_ERR_GATE_DRV

/**
 * 电机控制对外门面（VESC mc_interface / ODrive 协议层入口）。
 * main / 协议层只依赖本头文件；各轴 gate/PWM 在 motor_axis_init_hw 内完成。
 */
mc_error_t mc_init(const motor_system_config_t *config);
void mc_poll(void);
void mc_current_loop_isr(void);

void mc_set_ctrl_mode(uint8_t axis_id, control_mode_t mode);
control_mode_t mc_get_ctrl_mode(uint8_t axis_id);
fsm_state_t mc_get_fsm_state(uint8_t axis_id);

/** 请求轴进入 RUNNING；成功 MC_OK，失败见 MC_ERR_* */
mc_error_t mc_enable(uint8_t axis_id);
/** 请求轴停机（CTRL_MODE_IDLE + STATE_IDLE，关断 PWM） */
mc_error_t mc_disable(uint8_t axis_id);
/** 轴处于 RUNNING 且功率级已 arm */
bool mc_is_enabled(uint8_t axis_id);

/**
 * 运行时标定类型（与 fsm_state_t 中标定段一一对应，MC_CAL_SEQUENCE 为整链入口）。
 * 具体标定流程在 FSM on_enter/on_run 中实现，当前占位为自动步进。
 */
typedef enum
{
  MC_CAL_NONE = 0,
  MC_CAL_ENCODER,
  MC_CAL_RSLS,
  MC_CAL_FLUX,
  MC_CAL_ELECTRICAL_ALIGN,
  MC_CAL_ANTICOGGING,
  MC_CAL_SEQUENCE,
} mc_calibration_type_t;

/** 从 IDLE 启动标定；成功 MC_OK，失败见 MC_ERR_* */
int mc_start_calibration(uint8_t axis_id, mc_calibration_type_t type);
/** 中止标定并回到 STATE_IDLE（未在标定中则无操作） */
void mc_stop_calibration(uint8_t axis_id);
/** 是否处于标定 FSM 段（ENCODER_CALIBRATION … ANTICOGGING） */
bool mc_is_calibrating(uint8_t axis_id);
/** 当前标定类型；非标定中返回 MC_CAL_NONE */
mc_calibration_type_t mc_get_calibration_type(uint8_t axis_id);

/** 清除锁存故障并回 IDLE；成功 MC_OK，失败见 MC_ERR_* */
int mc_try_recover_from_fault(uint8_t axis_id);

#endif /* __MC_INTERFACE_H__ */
