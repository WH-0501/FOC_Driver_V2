#ifndef __FOC_H__
#define __FOC_H__

#include "datatypes.h"
#include "foc_fault.h"
#include "foc_math.h"
#include "common.h"

/** 控制栈初始化（config / PID / 零漂 / 可选电角对齐）；须先 motor_hw_init */
void foc_init(motor_handle_t *m, const motor_config_t *config);

void foc_control_loop(motor_handle_t *m);

/** 失能状态下执行电角对齐，结果写入 config.param.theta_elec_offset_rad。 */
error_t foc_align_electrical(motor_handle_t *m);

#endif /* __FOC_H__ */
