#ifndef __CURRENT_SENSE_H__
#define __CURRENT_SENSE_H__

#include "datatypes.h"

/*
 * 电流采样链路参数（ADC -> 电流换算）。
 */
#ifndef CURRENT_SENSE_ADC_VREF_V
#define CURRENT_SENSE_ADC_VREF_V (3.3f)
#endif
#ifndef CURRENT_SENSE_ADC_FULL_SCALE
#define CURRENT_SENSE_ADC_FULL_SCALE (4095.0f)
#endif
#ifndef CURRENT_SENSE_ADC_VOLTS_PER_LSB
#define CURRENT_SENSE_ADC_VOLTS_PER_LSB (CURRENT_SENSE_ADC_VREF_V / CURRENT_SENSE_ADC_FULL_SCALE)
#endif
#ifndef CURRENT_SENSE_SO_ISCALE
#define CURRENT_SENSE_SO_ISCALE (9200.0f)
#endif
#ifndef CURRENT_SENSE_SO_RTERM_OHM
#define CURRENT_SENSE_SO_RTERM_OHM (3300.0f)
#endif
#ifndef CURRENT_SENSE_SO_BIAS_V
#define CURRENT_SENSE_SO_BIAS_V (1.65f)
#endif
#ifndef CURRENT_SENSE_SO_AMPS_PER_VOLT
#define CURRENT_SENSE_SO_AMPS_PER_VOLT (CURRENT_SENSE_SO_ISCALE / CURRENT_SENSE_SO_RTERM_OHM)
#endif

/** 初始化并完成阻塞零漂校准（SimpleFOC current_sense.init 语义）。 */
void current_sense_init(motor_handle_t *m);
void current_sense_process_sample(motor_handle_t *m);

#endif /* __CURRENT_SENSE_H__ */
