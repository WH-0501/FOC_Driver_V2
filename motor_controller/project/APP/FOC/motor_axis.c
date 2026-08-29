#include "motor_axis.h"
#include "control_mode.h"
#include "foc_fault.h"
#include "encoder.h"
#include "foc_math.h"
#include "board.h"
#include "encoder/kth71xx.h"
#include <string.h>

#if (MOTOR_AXIS_ACTIVE_COUNT > MOTOR_AXIS_COUNT)
#error "MOTOR_AXIS_ACTIVE_COUNT must not exceed MOTOR_AXIS_COUNT"
#endif

static motor_axis_t s_motor_axes[MOTOR_AXIS_COUNT] = {
  {
    .id = 0u,
    .hw_present = true,
    .encoder = {
      [ENCODER_ROLE_SHAFT] = {
        .bind = { .dev = ENCODER_DEV_KTH71, .kth71_cs = KTH71_CS_0 },
      },
      [ENCODER_ROLE_JOINT] = {
        .bind = { .dev = ENCODER_DEV_NONE },
      },
    },
  },
  {
    .id = 1u,
    .hw_present = false,
    /* 占位轴：第二关节硬件就绪后设 hw_present=true，并填写 encoder / board 引脚 */
    .encoder = {
      [ENCODER_ROLE_SHAFT] = {
        .bind = { .dev = ENCODER_DEV_NONE },
      },
      [ENCODER_ROLE_JOINT] = {
        .bind = { .dev = ENCODER_DEV_NONE },
      },
    },
  },
};

bool motor_axis_is_present(const motor_axis_t *axis)
{
  return (axis != NULL) && axis->hw_present;
}

bool motor_axis_is_present_id(uint8_t axis_id)
{
  return motor_axis_is_present(motor_axis_get(axis_id));
}

motor_axis_t *motor_axis_get(uint8_t id)
{
  if (id >= MOTOR_AXIS_COUNT)
  {
    return NULL;
  }

  return &s_motor_axes[id];
}

static bool motor_axis_joint_encoder_enabled(const motor_axis_t *axis)
{
  if (axis == NULL)
  {
    return false;
  }

  return axis->encoder[ENCODER_ROLE_JOINT].bind.dev != ENCODER_DEV_NONE;
}

static void motor_axis_publish_encoder_state(motor_axis_t *axis)
{
  motor_handle_t *m;
  const encoder_unit_t *shaft;
  const encoder_unit_t *joint;
  float pole_pairs;
  float shaft_dir;
  float gear_ratio;
  bool shaft_ok;
  bool joint_ok;

  if (axis == NULL)
  {
    return;
  }

  m = &axis->handle;
  shaft = &axis->encoder[ENCODER_ROLE_SHAFT];
  joint = &axis->encoder[ENCODER_ROLE_JOINT];
  shaft_ok = shaft->bind.dev != ENCODER_DEV_NONE;
  joint_ok = motor_axis_joint_encoder_enabled(axis);

  pole_pairs = (m->config.param.pole_pairs > 0u) ? (float)m->config.param.pole_pairs : 1.0f;
  shaft_dir = (m->config.param.direction == -1) ? -1.0f : 1.0f;
  gear_ratio = (m->config.param.gear_ratio > 0.0f) ? m->config.param.gear_ratio : 1.0f;

  if (shaft_ok)
  {
    m->state.shaft = shaft->observer.sample;
    m->state.theta_elec_rad = angle_normalize(
        (shaft->observer.sample.angle_rad * pole_pairs * shaft_dir) -
        m->config.param.theta_elec_offset_rad);
  }

  if (joint_ok)
  {
    m->state.joint = joint->observer.sample;
  }
  else if (shaft_ok)
  {
    m->state.joint.counts = 0u;
    m->state.joint.angle_rad = 0.0f;
    m->state.joint.position_rad = shaft->observer.sample.position_rad / gear_ratio;
    m->state.joint.velocity_rad_s = shaft->observer.sample.velocity_rad_s / gear_ratio;
  }
}

static void motor_axis_init_encoders(motor_axis_t *axis)
{
  uint8_t i;

  if (axis == NULL)
  {
    return;
  }

  for (i = 0u; i < ENCODER_ROLE_COUNT; i++)
  {
    encoder_observer_init(&axis->encoder[i].observer);

    if (axis->encoder[i].bind.dev != ENCODER_DEV_NONE)
    {
      encoder_dev_init_once(axis->encoder[i].bind.dev);
    }
  }
}

error_t motor_axis_init_hw(motor_axis_t *axis)
{
  if (axis == NULL)
  {
    return ERR_FAIL;
  }

  if (!motor_axis_is_present(axis))
  {
    memset(&axis->driver, 0, sizeof(axis->driver));
    return ERR_NONE;
  }

  if (motor_driver_init(&axis->driver, axis->id) != ERR_NONE)
  {
    return ERR_FAIL;
  }

  motor_axis_init_encoders(axis);
  return ERR_NONE;
}

void motor_axis_update_encoders(motor_axis_t *axis)
{
  motor_handle_t *m;
  uint8_t i;
  uint32_t now_us;
  float default_zero;
  int8_t default_dir;

  if (axis == NULL)
  {
    return;
  }

  if (!motor_axis_is_present(axis))
  {
    return;
  }

  if (axis->encoder[ENCODER_ROLE_SHAFT].bind.dev == ENCODER_DEV_NONE)
  {
    return;
  }

  m = &axis->handle;
  now_us = board_time_us();
  default_zero = m->config.param.theta_offset_rad;
  default_dir = m->config.param.direction;

  for (i = 0u; i < ENCODER_ROLE_COUNT; i++)
  {
    (void)encoder_observer_update(&axis->encoder[i].observer,
                                  &axis->encoder[i].bind,
                                  (encoder_role_t)i,
                                  default_zero,
                                  default_dir,
                                  now_us);
  }

  motor_axis_publish_encoder_state(axis);
}

motor_axis_t *motor_axis_from_handle(motor_handle_t *m)
{
  uint8_t i;

  if (m == NULL)
  {
    return NULL;
  }

  for (i = 0u; i < MOTOR_AXIS_COUNT; i++)
  {
    if (m == &s_motor_axes[i].handle)
    {
      return &s_motor_axes[i];
    }
  }

  return NULL;
}

void motor_axis_arm(motor_handle_t *m)
{
  motor_axis_t *axis = motor_axis_from_handle(m);

  if (axis == NULL)
  {
    return;
  }

  if (!motor_axis_is_present(axis))
  {
    return;
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return;
  }

  motor_driver_arm(&axis->driver);

  m->out.pwm_enable = motor_driver_is_armed(&axis->driver) ? 1u : 0u;
}

/**
 * @brief 电机轴下电
 *        1. 输出PWM设置为0
 *        2. 电机驱动器下电(PWM关闭,预驱休眠,所有相位使能关闭)
 * @param m 
 * @return void 
 */
void motor_axis_disarm(motor_handle_t *m)
{
  motor_axis_t *axis = motor_axis_from_handle(m);

  if (axis == NULL)
  {
    return;
  }

  m->out.pwm_enable = 0u;

  motor_driver_disarm(&axis->driver);
}

void motor_axis_shutdown(motor_handle_t *m)
{
  motor_axis_t *axis = motor_axis_from_handle(m);

  if (axis == NULL)
  {
    return;
  }

  m->out.pwm_enable = 0u;
  motor_driver_shutdown(&axis->driver);
}

bool motor_axis_is_armed(const motor_handle_t *m)
{
  const motor_axis_t *axis = motor_axis_from_handle((motor_handle_t *)m);

  if (axis == NULL)
  {
    return false;
  }

  return motor_driver_is_armed(&axis->driver);
}

void motor_axis_sync_power(motor_handle_t *m)
{
  if (m == NULL)
  {
    return;
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return;
  }

  if (control_mode_needs_pwm(m->ctrl_mode))
  {
    if (!motor_axis_is_armed(m))
    {
      motor_axis_arm(m);
    }
  }
  else if (motor_axis_is_armed(m))
  {
    motor_axis_disarm(m);
  }
}

/**
 * @brief 进入直流偏移校准状态
 *        1. 输出PWM设置为0
 *        2. 准备进行直流偏移校准
 * @param m 
 * @return void 
 */
void motor_axis_enter_dc_cal_hw(motor_handle_t *m)
{
  motor_axis_t *axis = motor_axis_from_handle(m);

  if (axis == NULL)
  {
    return;
  }

  m->out.pwm_enable = 0u;
  motor_driver_prepare_dc_offset_cal(&axis->driver);
}

error_t motor_axis_apply_pwm(motor_handle_t *m, const motor_actuation_t *actuation)
{
  motor_axis_t *axis = motor_axis_from_handle(m);

  if ((axis == NULL) || (actuation == NULL) || !motor_axis_is_present(axis))
  {
    return ERR_FAIL;
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return ERR_FAIL;
  }

  if (!motor_driver_is_armed(&axis->driver))
  {
    return ERR_FAIL;
  }

  return motor_driver_set_pwm(&axis->driver, actuation);
}
