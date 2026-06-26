#ifndef __BOARD_H__
#define __BOARD_H__

#if !defined(PLATFORM_STM32) && !defined(PLATFORM_AT32)
#error "board.h: define PLATFORM_STM32 or PLATFORM_AT32 in project preprocessor symbols."
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "datatypes.h"
#include "error_types.h"

void board_init(motor_handle_t *m);
void board_deinit(void);

void board_get_phase_current(motor_handle_t *m);

void board_current_loop_irq_handler(void *adc_handle);
#define CURRENT_LOOP_IRQ_HANDLER board_current_loop_irq_handler

error_t current_hw_init(void);
error_t current_hw_deinit(void);
error_t pwm_hw_init(void);

/* PWM总开关接口 */
error_t pwm_hw_start(void);
error_t pwm_hw_stop(void);

error_t set_pwm(motor_actuation_t *actuation);

#endif /* __BOARD_H__ */
