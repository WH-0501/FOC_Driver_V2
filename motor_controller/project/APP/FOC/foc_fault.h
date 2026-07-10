#ifndef __FOC_FAULT_H__
#define __FOC_FAULT_H__

#include "error_types.h"
#include "datatypes.h"

void motor_fault_raise(motor_handle_t *m, fault_t bits);

void motor_fault_clear_status(motor_handle_t *m, fault_t mask);

void motor_fault_clear_latched(motor_handle_t *m, fault_t mask);

fault_t motor_fault_status_get(const motor_handle_t *m);

fault_t motor_fault_latched_get(const motor_handle_t *m);

int motor_fault_is_latched(const motor_handle_t *m);

/**
 * 故障保护：刷新瞬时故障位；锁存后由 mc_poll 调用 foc_on_fault_latched 关断。
 */
void motor_fault_protect(motor_handle_t *m);

#endif /* __FOC_FAULT_H__ */
