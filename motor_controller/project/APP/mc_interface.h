#ifndef __MC_INTERFACE_H__
#define __MC_INTERFACE_H__

#include "datatypes.h"
#include "motor_config.h"

typedef enum
{
  MC_OK = 0,

  MC_INIT_ERR_GATE_HW = -1,
  MC_INIT_ERR_GATE_DRV = -2,
  MC_INIT_ERR_GATE_PARAM = -3,
  MC_INIT_ERR_GATE_NOT_IMPL = -4,
  MC_INIT_ERR_CURRENT_LOOP = -5,
  MC_INIT_ERR_CONFIG = -6,

  MC_ERR_FAULT = 2,
  MC_ERR_OFFSET_CAL = 3,
  MC_ERR_CTRL_MODE = 4,
  MC_ERR_STATE = 5,
  MC_ERR_CALIBRATING = 6,
  MC_ERR_CAL_TYPE = 7,
  MC_ERR_NO_FAULT = 8,
} mc_error_t;

#define MC_INIT_OK MC_OK
#define MC_INIT_ERR_GATE MC_INIT_ERR_GATE_DRV

mc_error_t mc_init(const motor_config_t *config);
void mc_poll(void);
void mc_current_loop_isr(void);

void mc_set_ctrl_mode(control_mode_t mode);
control_mode_t mc_get_ctrl_mode(void);
fsm_state_t mc_get_fsm_state(void);

mc_error_t mc_enable(void);
mc_error_t mc_disable(void);
bool mc_is_enabled(void);

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

int mc_start_calibration(mc_calibration_type_t type);
void mc_stop_calibration(void);
bool mc_is_calibrating(void);
mc_calibration_type_t mc_get_calibration_type(void);

int mc_try_recover_from_fault(void);

/** 异常复位：关断输出、清除故障、清零给定与 PID，回到 IDLE。 */
int mc_reset_from_exception(void);

/** 失能状态下将当前轴端位置设为零点（更新 theta_offset_rad）。 */
int mc_set_joint_origin(void);

/** 失能状态下执行电角对齐。 */
int mc_align_electrical(void);

/**
 * KTH71 非线性自校准：须已使能（速度开环或电流/速度闭环）；不影响使能与 CAN 上报。
 */
int mc_kth71_cal_start(void);
/** 开环达速后由上位机调用，触发芯片 ANLC。 */
int mc_kth71_cal_trigger(void);
int mc_kth71_cal_stop(void);
/** 返回缓存的 ANLC 状态（不在 CAN 路径读寄存器）。 */
int mc_kth71_cal_get_status(uint8_t *status);
bool mc_kth71_cal_active(void);
/** mc_poll 内调用：节流刷新 ANLC 状态缓存（避免与读角争用 ISR）。 */
void mc_kth71_cal_service(void);

#endif /* __MC_INTERFACE_H__ */
