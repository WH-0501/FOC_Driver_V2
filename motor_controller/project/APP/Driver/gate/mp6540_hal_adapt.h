#ifndef __MP6540_HAL_ADAPT_H__
#define __MP6540_HAL_ADAPT_H__

#include "mp6540.h"
#if defined(PLATFORM_AT32)
#include "dwt_profile_delay.h"

void MP6540_HW_GPIO_Set(gpio_port_t port, gpio_pin_t pin, bool level)
{
    if (port == NULL)
    {
        return;
    }
    if (level)
    {
        gpio_bits_set(port, pin);
    }
    else
    {
        gpio_bits_reset(port, pin);
    }
}

bool MP6540_HW_GPIO_Read(gpio_port_t port, gpio_pin_t pin)
{
    if (port == NULL)
    {
        return false;
    }
    return (gpio_input_data_bit_read(port, pin) != RESET);
}

void MP6540_HW_DelayMs(uint32_t ms)
{
    delay_ms(ms);
}

#elif defined(PLATFORM_STM32)
#include "stm32h7xx_hal.h"

void MP6540_HW_GPIO_Set(gpio_port_t port, gpio_pin_t pin, bool level)
{
    if (port == NULL)
    {
        return;
    }
    HAL_GPIO_WritePin(port, pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool MP6540_HW_GPIO_Read(gpio_port_t port, gpio_pin_t pin)
{
    if (port == NULL)
    {
        return false;
    }
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET);
}

void MP6540_HW_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}
#endif

#endif /* __MP6540_HAL_ADAPT_H__ */
