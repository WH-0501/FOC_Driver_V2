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
  float align_theta_before_rad; ///< 对齐前电角（调试用）
  float align_theta_after_rad;  ///< 对齐锁定后电角（调试用，应≈align_angle）
  float align_raw_engage_rad;   ///< 转子吸合采样 raw 电角（未减 offset）
  float align_raw_elec_rad;     ///< 锁定瞬间 raw 电角（未减 offset）
  float align_polarity_test_iq; ///< 方向校准后的 +Vq 探针 i_q [A]
  bool align_direction_flipped; ///< 是否翻转了 param.direction
  bool align_ok;                ///< 对齐+极性校验是否通过
  bool align_engaged;           ///< 转子是否吸合到位

  float foc_pos_loop_us;        ///< 位置环耗时 [us]（本周期未运行则为 0）
  float foc_vel_loop_us;        ///< 速度环耗时 [us]
  float foc_cur_loop_us;        ///< 电流环耗时 [us]
  float foc_ctrl_loop_us;       ///< foc_control_run 总耗时 [us]
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
