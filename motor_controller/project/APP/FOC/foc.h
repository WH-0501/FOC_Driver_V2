#ifndef __FOC_H__
#define __FOC_H__

#include "motor_axis.h"
#include "foc_fault.h"
#include "foc_math.h"
#include "common.h"

/** 单轴控制栈初始化（config / PID / 零漂 / 可选电角对齐）；须先 motor_axis_init_hw */
void foc_init_axis(motor_axis_t *axis, const motor_config_t *config);

void foc_control_loop(motor_handle_t *m);

#endif /* __FOC_H__ */
