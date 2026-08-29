#ifndef __MOTOR_AXIS_H__
#define __MOTOR_AXIS_H__

#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"
#include "encoder_types.h"
#include "error_types.h"
#include "motor_driver.h"

#ifndef MOTOR_AXIS_COUNT
#define MOTOR_AXIS_COUNT 2u
#endif

/** 已接线、可运行的轴数（≤ MOTOR_AXIS_COUNT）；其余槽位 hw_present=false 占位 */
#ifndef MOTOR_AXIS_ACTIVE_COUNT
#define MOTOR_AXIS_ACTIVE_COUNT 1u
#endif

/**
 * motor_axis_t：单轴运行时实例。
 * - hw_present：false 表示占位轴（无门驱/PWM/编码器，不参与 ISR）
 * - handle：控制模型（config / ref / state / pid / fsm）
 * - driver：本轴 MP6540 门驱 + PWM（仅 hw_present 时初始化）
 * - encoder[]：SHAFT + JOINT
 */

typedef struct
{
  uint32_t offset_sum[3];
  uint32_t offset_cnt;
  uint8_t offset_active;

  float open_loop_uq;
  float open_loop_omega;
  float open_loop_shaft_angle;
} motor_axis_runtime_t;

typedef struct
{
  uint8_t id;
  bool hw_present;
  motor_handle_t handle;
  motor_driver_t driver;
  encoder_unit_t encoder[ENCODER_ROLE_COUNT];
  motor_axis_runtime_t rt;
  uint8_t fsm_enter_pending;
  bool electrical_align_done;
} motor_axis_t;

motor_axis_t *motor_axis_get(uint8_t id);
motor_axis_t *motor_axis_from_handle(motor_handle_t *m);

/** 本轴是否已接线并参与控制（占位轴返回 false） */
bool motor_axis_is_present(const motor_axis_t *axis);
bool motor_axis_is_present_id(uint8_t axis_id);

/** 本轴功率级（门驱 + PWM）+ encoder[]；不含控制/config */
error_t motor_axis_init_hw(motor_axis_t *axis);

/** 更新本轴全部编码器，并写入 axis->handle.state */
void motor_axis_update_encoders(motor_axis_t *axis);

void motor_axis_arm(motor_handle_t *m);
void motor_axis_disarm(motor_handle_t *m);
void motor_axis_shutdown(motor_handle_t *m);
bool motor_axis_is_armed(const motor_handle_t *m);
void motor_axis_sync_power(motor_handle_t *m);
void motor_axis_enter_dc_cal_hw(motor_handle_t *m);
error_t motor_axis_apply_pwm(motor_handle_t *m, const motor_actuation_t *actuation);

#endif /* __MOTOR_AXIS_H__ */
