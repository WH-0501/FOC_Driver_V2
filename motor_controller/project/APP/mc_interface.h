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

/** 失能状态下将当前轴端位置设为零点（更新 theta_offset_rad）。 */
int mc_set_joint_origin(void);

/** 失能状态下执行电角对齐。 */
int mc_align_electrical(void);

#endif /* __MC_INTERFACE_H__ */
