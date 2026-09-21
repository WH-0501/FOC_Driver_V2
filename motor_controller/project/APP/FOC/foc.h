#ifndef __FOC_H__
#define __FOC_H__

#include "datatypes.h"
#include "foc_fault.h"
#include "foc_math.h"
#include "common.h"

/** 控制栈初始化（config / PID / 零漂 / 可选电角对齐）；须先 motor_hw_init */
void foc_init(motor_handle_t *m, const motor_config_t *config);

void foc_control_loop(motor_handle_t *m);

/** 开环电压输出（校准/对齐等慢路径使用）。 */
void foc_voltage(motor_handle_t *m, float uq, float ud, float angle_el);

#endif /* __FOC_H__ */
