#ifndef __PLATFORM_TYPES_H__
#define __PLATFORM_TYPES_H__

#include <stdint.h>

#if defined(PLATFORM_AT32)
#include "at32m412_416_gpio.h"
typedef gpio_type *gpio_port_t;
typedef uint16_t gpio_pin_t;
#elif defined(PLATFORM_STM32)
#include "stm32h7xx_hal.h"
typedef GPIO_TypeDef *gpio_port_t;
typedef uint16_t gpio_pin_t;
#else
#error "platform_types.h: define PLATFORM_AT32 or PLATFORM_STM32."
#endif

#endif /* __PLATFORM_TYPES_H__ */
