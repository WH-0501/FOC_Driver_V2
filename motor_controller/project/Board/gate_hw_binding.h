#ifndef __GATE_HW_BINDING_H__
#define __GATE_HW_BINDING_H__

#include "platform_gpio_types.h"

/**
 * 预驱 GPIO 绑定（板级事实，Board 与 gate_driver 共用，避免重复 struct / 字段拷贝）。
 */
typedef struct
{
  gpio_port_t port_nSLEEP;
  gpio_pin_t pin_nSLEEP;
  gpio_port_t port_ENA;
  gpio_pin_t pin_ENA;
  gpio_port_t port_ENB;
  gpio_pin_t pin_ENB;
  gpio_port_t port_ENC;
  gpio_pin_t pin_ENC;
  gpio_port_t port_FAULT;
  gpio_pin_t pin_FAULT;
} gate_hw_binding_t;

#endif /* __GATE_HW_BINDING_H__ */
