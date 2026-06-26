#include <math.h>
#include "foc_fault.h"
#include "math_compat.h"
#include "board.h"
#include "Driver/gate/gate_driver.h"

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

/**
 * @brief 故障保护：根据状态与配置刷新瞬时故障位并锁存
 *
 * @param m 电机句柄
 * @return void
 */
void motor_fault_protect(motor_handle_t *m)
{
  const motor_limits_t *L;
  motor_state_t *S;
  float i_abs_max;
  float a0, a1, a2, mx, mn, i_avg, imb;

  if (m == NULL)
  {
    return;
  }

  L = &m->config.limits;
  S = &m->state;

  m->fault.status &= ~FAULT_POLL_MASK;

  /* nFault 异常 */
  if (gate_driver_read_fault(gate_driver_get_active()))
  {
    motor_fault_raise(m, FAULT_GATE_DRIVER);
  }

  /* 母线欠压 */
  if (L->vbus_uv_threshold > 0.f && S->vbus > 0.f && S->vbus < L->vbus_uv_threshold)
  {
    motor_fault_raise(m, FAULT_BUS_UV);
  }

  /* 母线过压 */
  if (L->vbus_ov_threshold > 0.f && S->vbus > L->vbus_ov_threshold)
  {
    motor_fault_raise(m, FAULT_BUS_OV);
  }

  /* 软件过流 */
  if (L->sw_ocp > 0.f)
  {
    i_abs_max = FMAX(fabsf(S->phase_current.ampere[0]),
                     FMAX(fabsf(S->phase_current.ampere[1]), fabsf(S->phase_current.ampere[2])));
    if (i_abs_max > L->sw_ocp)
    {
      motor_fault_raise(m, FAULT_SW_OVERCURRENT);
    }
  }

  /* 电机过温 */
  if (L->temp_limit > 0.f && S->temperature > L->temp_limit)
  {
    motor_fault_raise(m, FAULT_OVERTEMP_MOTOR);
  }

  /* 超速 */
  if (m->config.motion.max_speed_rad_s > 0.f &&
      fabsf(S->speed_rad_s) > m->config.motion.max_speed_rad_s)
  {
    motor_fault_raise(m, FAULT_OVER_SPEED);
  }

  /* 缺相 */
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

  /* 如果已锁存任一故障，默认拉低 pwm_enable */
  if (motor_fault_is_latched(m))
  {
    const gate_driver_t *drv = gate_driver_get_active();
    m->out.pwm_enable = 0u;
    (void)pwm_hw_stop();
    /* gate driver 进入非激活态（关相使能并进入休眠等）。 */
    gate_driver_enter_inactive_state(drv);
  }
}
