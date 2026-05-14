#ifndef __BOARD_H__
#define __BOARD_H__

#if !defined(PLATFORM_STM32) && !defined(PLATFORM_AT32)
#error "board.h: define PLATFORM_STM32 or PLATFORM_AT32 in project preprocessor symbols."
#endif

#include <stdbool.h>
#include "datatypes.h"


/*
 * ADC 数字量 → 引脚电压：与 README「电流采样」一致时，相电流通道为
 *   V_SO = V_BIAS + (R_TERM * I_LOAD) / I_SCALE ，其中 I_SCALE=9200，R_TERM、V_BIAS 见下宏。
 * 运行态：用零电流时学到的 adc_offset 作偏置；校准态：用标称 V_BIAS（1.65V）作偏置。
 */
#ifndef BOARD_ADC_VREF_V
#define BOARD_ADC_VREF_V (3.3f)
#endif
#ifndef BOARD_ADC_FULL_SCALE
#define BOARD_ADC_FULL_SCALE (4095.0f)
#endif

#ifndef BOARD_ADC_VOLTS_PER_LSB
#define BOARD_ADC_VOLTS_PER_LSB (BOARD_ADC_VREF_V / BOARD_ADC_FULL_SCALE)
#endif

/** 电流镜比例 I_LOAD : I_SO，README 为 9200 */
#ifndef BOARD_SO_ISCALE
#define BOARD_SO_ISCALE (9200.0f)
#endif
/** 终端电阻 R_REF (Ω)，README 示例 3300Ω */
#ifndef BOARD_SO_RTERM_OHM
#define BOARD_SO_RTERM_OHM (3300.0f)
#endif
/** SOx 静态偏置电压 V_REF (V)，README 分压 1.65V */
#ifndef BOARD_SO_BIAS_V
#define BOARD_SO_BIAS_V (1.65f)
#endif

/** I = ΔV * (I_SCALE / R_TERM)，单位 A/V */
#ifndef BOARD_SO_AMPS_PER_VOLT
#define BOARD_SO_AMPS_PER_VOLT (BOARD_SO_ISCALE / BOARD_SO_RTERM_OHM)
#endif


void board_init(void);
void board_deinit(void);

void get_phase_current(void);

void board_apply_phase_current(motor_state_t *state);

void board_current_offset_cal_fsm_step(motor_handle_t *m);

void board_current_loop_irq_handler(void *adc_handle);
#define CURRENT_LOOP_IRQ_HANDLER board_current_loop_irq_handler

#endif /* __BOARD_H__ */
