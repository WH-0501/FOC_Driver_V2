#ifndef __MOTOR_AXIS_H__
#define __MOTOR_AXIS_H__

#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"
#include "error_types.h"

#ifndef MOTOR_AXIS_COUNT
#define MOTOR_AXIS_COUNT 1u
#endif

/**
 * 单轴实例（ODrive Axis）：运行态 + 轴级元数据。
 * motor_handle_t 仍为控制环/FSM 的核心状态体；多轴扩展时复制本结构。
 */
typedef struct
{
  uint8_t id;
  motor_handle_t handle;
  uint8_t fsm_enter_pending;
  bool electrical_align_done;
} motor_axis_t;

motor_axis_t *motor_axis_get(uint8_t id);
motor_axis_t *motor_axis_from_handle(motor_handle_t *m);

/** 单轴便捷访问：g_motor 等价于 motor_axis_get(0)->handle */
#define g_motor (motor_axis_get(0u)->handle)

/* --- 功率级（唯一入口：Control / current_sense / fault → axis → driver） --- */
void motor_axis_arm(motor_handle_t *m);
void motor_axis_disarm(motor_handle_t *m);
void motor_axis_shutdown(motor_handle_t *m);
bool motor_axis_is_armed(const motor_handle_t *m);
void motor_axis_sync_power(motor_handle_t *m);

void motor_axis_prepare_dc_cal(motor_handle_t *m);
error_t motor_axis_set_pwm(motor_handle_t *m, const motor_actuation_t *actuation);

#endif /* __MOTOR_AXIS_H__ */
