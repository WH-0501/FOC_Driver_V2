#ifndef __CONTROL_MODE_H__
#define __CONTROL_MODE_H__

#include <stdbool.h>
#include "datatypes.h"

/** 控制模式是否需要输出 PWM（FSM / motor_axis 共用，单点定义）。 */
static inline bool control_mode_needs_pwm(control_mode_t mode)
{
  switch (mode)
  {
    case CTRL_MODE_TORQUE:
    case CTRL_MODE_SPEED:
    case CTRL_MODE_POSITION:
    case CTRL_MODE_CSP:
    case CTRL_MODE_CSV:
    case CTRL_MODE_CST:
    case CTRL_MODE_MIT:
    case CTRL_MODE_HOMING:
    case CTRL_MODE_TORQUE_OPEN_LOOP:
    case CTRL_MODE_SPEED_OPEN_LOOP:
    case CTRL_MODE_POSITION_OPEN_LOOP:
    case CTRL_MODE_VOLTAGE:
      return true;
    default:
      return false;
  }
}

#endif /* __CONTROL_MODE_H__ */
