#include "motor_driver.h"
#include "board.h"

/**
 * @brief 绑定电机驱动器
 *        1. 获取硬件配置
 *        2. 初始化电机驱动器(进入休眠状态)
 *        3. 绑定电机驱动器
 * @param drv 电机驱动器
 * @return error_t 错误码
 */
static error_t motor_driver_bind_gate(motor_driver_t *drv)
{
  gate_driver_init_t cfg;

  if (drv == NULL)
  {
    return ERR_FAIL;
  }

  cfg.type = GATE_DRIVER_TYPE_MP6540;
  if (board_get_gate_hw(&cfg.hw) != 0)
  {
    drv->gate = NULL;
    return ERR_FAIL;
  }

  MP6540_Init(&drv->mp6540, &cfg.hw);
  drv->gate = MP6540_AsGateDriver(&drv->mp6540);
  if (drv->gate == NULL)
  {
    return ERR_FAIL;
  }

  return ERR_NONE;
}

error_t motor_driver_init(motor_driver_t *drv)
{
  if (drv == NULL)
  {
    return ERR_FAIL;
  }

  drv->armed = false;
  drv->gate = NULL;

  if (motor_driver_bind_gate(drv) != ERR_NONE)
  {
    return ERR_FAIL;
  }

  motor_driver_shutdown(drv);
  return ERR_NONE;
}

const gate_driver_t *motor_driver_get_gate(const motor_driver_t *drv)
{
  if (drv == NULL)
  {
    return NULL;
  }

  return drv->gate;
}

void motor_driver_arm(motor_driver_t *drv)
{
  if ((drv == NULL) || (drv->gate == NULL) || drv->armed)
  {
    return;
  }

  (void)pwm_hw_init();
  gate_driver_exit_sleep(drv->gate);
  gate_driver_set_all_phase_en(drv->gate, true);
  (void)pwm_hw_start();
  drv->armed = true;
}

/**
 * @brief 关闭电机驱动器 （停机）
 * 
 * 1. 停止 PWM 输出
 * 2. 退出睡眠模式
 * 3. 设置所有相位使能为 false
 * 4. 设置电机驱动器状态为未使能
 * 
 * @param drv: 电机驱动器
 */
void motor_driver_disarm(motor_driver_t *drv)
{
  if ((drv == NULL) || (drv->gate == NULL))
  {
    return;
  }

  (void)pwm_hw_stop();
  gate_driver_exit_sleep(drv->gate);
  gate_driver_set_all_phase_en(drv->gate, false);
  drv->armed = false;
}

void motor_driver_shutdown(motor_driver_t *drv)
{
  if ((drv == NULL) || (drv->gate == NULL))
  {
    return;
  }

  (void)pwm_hw_stop();
  gate_driver_enter_inactive_state(drv->gate);
  drv->armed = false;
}

bool motor_driver_is_armed(const motor_driver_t *drv)
{
  if (drv == NULL)
  {
    return false;
  }

  return drv->armed;
}

void motor_driver_prepare_dc_offset_cal(motor_driver_t *drv)
{
  if ((drv == NULL) || (drv->gate == NULL))
  {
    return;
  }

  (void)pwm_hw_init();
  gate_driver_exit_sleep(drv->gate);
  gate_driver_set_all_phase_en(drv->gate, true);
  (void)pwm_hw_lowside_brake_on();
  (void)pwm_hw_start();
  drv->armed = false;
}

error_t motor_driver_set_pwm(motor_driver_t *drv, const motor_actuation_t *actuation)
{
  if ((drv == NULL) || !drv->armed)
  {
    return ERR_FAIL;
  }

  if (actuation == NULL)
  {
    return ERR_FAIL;
  }

  return set_pwm((motor_actuation_t *)actuation);
}
