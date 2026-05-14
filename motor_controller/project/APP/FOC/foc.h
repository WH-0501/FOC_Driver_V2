#ifndef __FOC_H__
#define __FOC_H__

#include "board.h"
#include "error.h"
#include "foc_math.h"
#include "common.h"

/** 单轴 FOC 根实例：配置/给定/状态/输出/FSM/PID 均经此访问 */
extern motor_handle_t g_motor;

void foc_init(void);
void foc_update(void);

void foc_current_loop_control(void);

void foc_pos_vel_loop_control(void);

void foc_state_machine_loop(void);

#endif /* __FOC_H__ */
