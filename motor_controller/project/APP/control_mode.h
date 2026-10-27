#ifndef __CONTROL_MODE_H__
#define __CONTROL_MODE_H__

#include <stdbool.h>
#include "datatypes.h"

/* Implemented modes: API admission, PWM gating and FOC dispatch share this list.
 * Other control_mode_t values may exist for protocol reserve; they are not implemented. */
static inline bool control_mode_is_implemented(control_mode_t mode)
{
  switch (mode)
  {
    case CTRL_MODE_IDLE:
    case CTRL_MODE_TORQUE:
    case CTRL_MODE_SPEED:
    case CTRL_MODE_POSITION:
    case CTRL_MODE_SPEED_OPEN_LOOP:
      return true;
    default:
      return false;
  }
}

static inline bool control_mode_is_supported(control_mode_t mode)
{
  return control_mode_is_implemented(mode);
}

static inline bool control_mode_needs_pwm(control_mode_t mode)
{
  return control_mode_is_implemented(mode) && (mode != CTRL_MODE_IDLE);
}

#endif /* __CONTROL_MODE_H__ */
