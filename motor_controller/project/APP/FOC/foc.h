#ifndef __FOC_H__
#define __FOC_H__

#include "board.h"

extern motor_state_t g_motor_state;

void foc_init(void);
void foc_update(void);

void foc_current_loop_control(void);

void foc_pos_vel_loop_control(void);

void foc_state_machint_loop(void);

#endif /* __FOC_H__ */
