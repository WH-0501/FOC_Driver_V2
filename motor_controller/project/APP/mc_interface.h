#ifndef __MC_INTERFACE_H__
#define __MC_INTERFACE_H__

#include "datatypes.h"
#include "motor_axis.h"

enum
{
  MC_INIT_OK = 0,
  MC_INIT_ERR_GATE = -1,
};

/**
 * 电机控制对外门面（VESC mc_interface / ODrive 协议层入口）。
 * main / 协议层只依赖本头文件；gate_driver、motor_driver、board 绑定在 mc_init 内完成。
 */
int mc_init(const motor_config_t *config);
void mc_poll(void);
void mc_current_loop_isr(void);

void mc_set_ctrl_mode(uint8_t axis_id, control_mode_t mode);
control_mode_t mc_get_ctrl_mode(uint8_t axis_id);
fsm_state_t mc_get_fsm_state(uint8_t axis_id);

int mc_try_recover_from_fault(uint8_t axis_id);

#endif /* __MC_INTERFACE_H__ */
