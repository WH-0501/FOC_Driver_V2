#include <math.h>
#include "foc_fault.h"
#include "foc_fsm.h"
#include "math_compat.h"
#include "motor.h"
#include "common.h"
#include "gate/gate_driver.h"

/* Independent shaft-speed absolute protection. Never uses gear ratio or a
 * writable runtime ceiling. The caller must disarm immediately on true. */
bool motor_fault_check_shaft_speed(motor_handle_t *m)
{
  float w;
  if (m == NULL) return false;
  w = m->state.shaft.velocity_rad_s;
  if (!(w <= 0.0f || w >= 0.0f)) {
    motor_fault_raise(m, FAULT_ENCODER);
    return true;
  }
  if (fabsf(w) >= MAX_VELOCITY_RAD_S) {
    motor_fault_raise(m, FAULT_OVER_SPEED);
    return true;
  }
  return false;
}

void motor_fault_raise(motor_handle_t *m, fault_t bits)
{
  if (m == NULL || bits == FAULT_NONE)
  {
    return;
  }
  m->fault.status |= bits;
  m->fault.latched |= bits;
}

void motor_fault_clear_status(motor_handle_t *m, fault_t mask)
{
  if (m == NULL)
  {
    return;
  }
  m->fault.status &= ~mask;
}

void motor_fault_clear_latched(motor_handle_t *m, fault_t mask)
{
  if (m == NULL)
  {
    return;
  }
  m->fault.latched &= ~mask;
  m->fault.status &= ~mask;
}

fault_t motor_fault_status_get(const motor_handle_t *m)
{
  if (m == NULL)
  {
    return FAULT_NONE;
  }
  return m->fault.status;
}

fault_t motor_fault_latched_get(const motor_handle_t *m)
{
  if (m == NULL)
  {
    return FAULT_NONE;
  }
  return m->fault.latched;
}

int motor_fault_is_latched(const motor_handle_t *m)
{
  return (m != NULL && m->fault.latched != FAULT_NONE);
}

void motor_fault_protect(motor_handle_t *m)
{
  const motor_limits_t *L;
  motor_state_t *S;
  float vbus_mon;
  float i_abs_max;
  float a0, a1, a2, mx, mn, i_avg, imb;

  const gate_driver_t *gate;

  if (m == NULL)
  {
    return;
  }

  L = &m->config.limits;
  S = &m->state;

  vbus_mon = S->vbus_filtered;
  if (vbus_mon <= 0.f)
  {
    board_get_vbus_and_temp(m);
    vbus_mon = S->vbus;
  }

  // 先清除POLL_MASK掩码位，再进行故障检测
  m->fault.status &= ~FAULT_POLL_MASK;

  gate = motor_gate(m);

  if ((gate != NULL) && gate_driver_read_fault(gate))
  {
    motor_fault_raise(m, FAULT_GATE_DRIVER);
  }

  if (L->vbus_uv_threshold > 0.f && vbus_mon > 0.f && vbus_mon < L->vbus_uv_threshold)
  {
    motor_fault_raise(m, FAULT_BUS_UV);
  }

  if (L->vbus_ov_threshold > 0.f && vbus_mon > L->vbus_ov_threshold)
  {
    motor_fault_raise(m, FAULT_BUS_OV);
  }

  if (L->sw_ocp > 0.f)
  {
    i_abs_max = FMAX(fabsf(S->phase_current.ampere[0]),
                     FMAX(fabsf(S->phase_current.ampere[1]), fabsf(S->phase_current.ampere[2])));
    if (i_abs_max > L->sw_ocp)
    {
      motor_fault_raise(m, FAULT_SW_OVERCURRENT);
    }
  }

  if (L->temp_limit > 0.f && S->board_temp.value > L->temp_limit)
  {
    motor_fault_raise(m, FAULT_OVERTEMP_MOTOR);
  }

  (void)motor_fault_check_shaft_speed(m);

  if (L->phase_diag_i_avg_min_a > 0.f &&
      (L->phase_imbalance_ratio_max > 0.f || L->phase_imbalance_warn_ratio > 0.f))
  {
    a0 = fabsf(S->phase_current.ampere[0]);
    a1 = fabsf(S->phase_current.ampere[1]);
    a2 = fabsf(S->phase_current.ampere[2]);
    mx = FMAX(a0, FMAX(a1, a2));
    mn = FMIN(a0, FMIN(a1, a2));
    i_avg = (a0 + a1 + a2) * (1.0f / 3.0f);
    if (i_avg > L->phase_diag_i_avg_min_a && i_avg > 1e-6f)
    {
      imb = (mx - mn) / i_avg;
      if (L->phase_imbalance_ratio_max > 0.f && imb > L->phase_imbalance_ratio_max)
      {
        motor_fault_raise(m, FAULT_PHASE_LOSS);
      }
      else if (L->phase_imbalance_warn_ratio > 0.f && imb > L->phase_imbalance_warn_ratio)
      {
        motor_fault_raise(m, FAULT_CURRENT_IMBALANCE);
      }
    }
  }

  if (motor_fault_is_latched(m))
  {
    foc_on_fault_latched(m);
  }
}
