/**
 * @file motor_fault.h
 * @brief 经典 FOC/伺服风格故障：位掩码 + 锁存，供 ISR/慢任务/上位机解析。
 */
#ifndef __ERROR_H__
#define __ERROR_H__

#include "error_types.h"
#include "datatypes.h"

void motor_fault_raise(motor_handle_t *m, fault_t bits);

void motor_fault_clear_status(motor_handle_t *m, fault_t mask);

void motor_fault_clear_latched(motor_handle_t *m, fault_t mask);

fault_t motor_fault_status_get(const motor_handle_t *m);

fault_t motor_fault_latched_get(const motor_handle_t *m);

int motor_fault_is_latched(const motor_handle_t *m);

/**
 * 故障保护：根据 state 与 config.limits 刷新瞬时故障位并锁存；
 * 若已锁存任一故障，默认拉低 pwm_enable（需在清除锁存后由应用重新使能）。
 */
void motor_fault_protect(motor_handle_t *m);

#endif /* __ERROR_H__ */
