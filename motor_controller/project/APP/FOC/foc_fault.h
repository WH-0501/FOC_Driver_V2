#ifndef __FOC_FAULT_H__
#define __FOC_FAULT_H__

#include "error_types.h"
#include "datatypes.h"

/* Shaft-domain speed limiter. Soft=0 disables only the soft limiter.
 * Full user torque below 90%; linearly remove accelerating torque by 100%.
 * Braking torque is preserved in both directions. No active brake is invented. */
static float speed_limit_soft_iq(float iq, float shaft_rad_s, float soft_rad_s)
{
  float speed, scale;
  if (!(iq <= 0.0f || iq >= 0.0f) ||
      !(shaft_rad_s <= 0.0f || shaft_rad_s >= 0.0f)) return 0.0f;
  if (!(soft_rad_s > 0.0f)) return iq; // 限速值为0，则不限制
  if (!((iq > 0.0f && shaft_rad_s > 0.0f) ||
        (iq < 0.0f && shaft_rad_s < 0.0f))) return iq;
  speed = fabsf(shaft_rad_s);
  /** 电机轴速度超过限速值，则限制为限速值 */
  if (speed >= soft_rad_s) return 0.0f;
  /** 电机轴速度低于限速值的85%，则不限制 */
  if (speed <= 0.85f * soft_rad_s) return iq;
  /** 电机轴速度在限速值的85%到100%之间，则线性限制 */
  scale = (soft_rad_s - speed) / (0.1f * soft_rad_s);
  return iq * scale;
}

/** Fixed shaft-speed protection; latches fault, caller must disarm on true. */
bool motor_fault_check_shaft_speed(motor_handle_t *m);

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
