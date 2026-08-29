#ifndef __FOC_FSM_H__
#define __FOC_FSM_H__

#include "datatypes.h"

void foc_fsm_init(motor_handle_t *m);
void foc_fsm_next_state(motor_handle_t *m, fsm_state_t next_state);
void foc_fsm_loop(motor_handle_t *m);

fsm_state_t foc_fsm_get_state(const motor_handle_t *m);

/** 从 IDLE 请求进入运行（enable）；已完成上电标定则直达 RUNNING */
void foc_fsm_request_running(motor_handle_t *m);

/** 从 IDLE 进入指定标定状态（ENCODER_CALIBRATION … ANTICOGGING） */
void foc_fsm_request_calibration(motor_handle_t *m, fsm_state_t entry_state);

/** 是否处于标定 FSM 段 */
bool foc_fsm_is_calibrating(fsm_state_t state);

/** 故障锁存后由 foc_fault 调用：关断功率级并进入 STATE_FAULT（幂等）。 */
void foc_on_fault_latched(motor_handle_t *m);

#endif /* __FOC_FSM_H__ */
