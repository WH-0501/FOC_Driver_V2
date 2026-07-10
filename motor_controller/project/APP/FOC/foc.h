#ifndef __FOC_H__
#define __FOC_H__

#include "motor_axis.h"
#include "foc_fault.h"
#include "foc_math.h"
#include "common.h"

void foc_init(const motor_config_t *config);

void foc_control_loop(motor_handle_t *m);
void foc_motor_run(void);

void foc_pos_vel_loop_control(void);

#endif /* __FOC_H__ */
