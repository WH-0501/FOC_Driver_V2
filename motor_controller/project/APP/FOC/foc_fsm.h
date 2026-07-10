#ifndef __FOC_FSM_H__
#define __FOC_FSM_H__

#include "datatypes.h"

void foc_fsm_init(motor_handle_t *m);
void foc_fsm_next_state(motor_handle_t *m, fsm_state_t next_state);
void foc_fsm_loop(motor_handle_t *m);

fsm_state_t foc_fsm_get_state(const motor_handle_t *m);

/** 故障锁存后由 foc_fault 调用：关断功率级并进入 STATE_FAULT（幂等）。 */
void foc_on_fault_latched(motor_handle_t *m);

#endif /* __FOC_FSM_H__ */
