#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"
#include "error_types.h"
#include "gate/gate_driver.h"

typedef struct
{
  uint32_t offset_sum[3];
  uint32_t offset_cnt;
  uint8_t offset_active;

  float open_loop_uq;           ///< 开环 Uq 参考值
  float open_loop_omega;        ///< 开环转速参考值 [rad/s]
  float open_loop_shaft_angle;  ///< 开环转子机械角参考值 [rad]

  uint8_t fsm_enter_pending;    ///< FSM 进入等待标志
  bool electrical_align_done;   ///< 电气对齐完成标志
} motor_runtime_t;

motor_handle_t *motor_get_handler(void);
motor_runtime_t *motor_runtime(motor_handle_t *m);
const gate_driver_t *motor_gate(motor_handle_t *m);

error_t motor_hw_init(void);

void motor_arm(motor_handle_t *m);
void motor_disarm(motor_handle_t *m);
void motor_shutdown(motor_handle_t *m);
bool motor_is_armed(const motor_handle_t *m);
void motor_sync_arm_with_mode(motor_handle_t *m);
void motor_enter_dc_cal_hw(motor_handle_t *m);
error_t motor_apply_pwm(motor_handle_t *m, const motor_actuation_t *actuation);

#endif /* __MOTOR_H__ */
