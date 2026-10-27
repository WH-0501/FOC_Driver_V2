#ifndef __MC_INTERFACE_H__
#define __MC_INTERFACE_H__

#include "datatypes.h"
#include "motor_config.h"
#include "FOC/foc_calibration.h"

/** 从存储加载配置并初始化电机控制器（配置写入 motor_get_handler()->config）。 */
error_t mc_init(void);
void mc_poll(void);
void mc_current_loop_isr(void);

/** Only implemented modes are accepted. Non-idle changes require STATE_IDLE
 * and disarmed hardware; IDLE requests stop. Returns ERR_CTRL_MODE for an
 * unsupported mode. Call from the foreground, not from a control ISR. */
error_t mc_set_ctrl_mode(control_mode_t mode);
control_mode_t mc_get_ctrl_mode(void);
fsm_state_t mc_get_fsm_state(void);

error_t mc_enable(void);
error_t mc_disable(void);
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

error_t mc_start_calibration(mc_calibration_type_t type);
void mc_stop_calibration(void);
bool mc_is_calibrating(void);
mc_calibration_type_t mc_get_calibration_type(void);

error_t mc_try_recover_from_fault(void);

/** 异常复位：关断输出、清除故障、清零给定与 PID，回到 IDLE。 */
error_t mc_reset_from_exception(void);

/** 失能状态下将当前轴端位置设为零点（更新 theta_offset_rad）。 */
error_t mc_set_joint_origin(void);

/** 失能状态下执行电角对齐；method 传 CAL_ELEC_ALIGN_METHOD_DEFAULT 为默认方案。 */
error_t mc_align_electrical(cal_elec_align_method_t method);

/**
 * KTH71 非线性自校准：须已使能（速度开环或电流/速度闭环）；不影响使能与 CAN 上报。
 */
error_t mc_kth71_cal_start(void);
/** 开环达速后由上位机调用，触发芯片 ANLC。 */
error_t mc_kth71_cal_trigger(void);
error_t mc_kth71_cal_stop(void);
/** 返回缓存的 ANLC 状态（不在 CAN 路径读寄存器）。 */
error_t mc_kth71_cal_get_status(uint8_t *status);
bool mc_kth71_cal_active(void);
/** mc_poll 内调用：节流刷新 ANLC 状态缓存（避免与读角争用 ISR）。 */
void mc_kth71_cal_service(void);

#endif /* __MC_INTERFACE_H__ */
