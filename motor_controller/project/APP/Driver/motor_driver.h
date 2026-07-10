#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#include <stdbool.h>
#include "datatypes.h"
#include "error_types.h"

/**
 * 逆变器功率级驱动（gate + PWM + 电流采样 IRQ 编排）。
 * 与 motor_axis 分工：本层管 HW 组合原语，axis 管策略与 handle 绑定。
 */
void motor_driver_init(void);
void motor_driver_arm(void);
void motor_driver_disarm(void);
void motor_driver_shutdown(void);
bool motor_driver_is_armed(void);

void motor_driver_prepare_dc_offset_cal(void);
error_t motor_driver_set_pwm(const motor_actuation_t *actuation);

#endif /* __MOTOR_DRIVER_H__ */
