#ifndef AT32_FOC_TIMING_H
#define AT32_FOC_TIMING_H

/* One timing contract for PWM, injected ADC and the output guard.
 * PWM A active-high: CNT > CCR means MP6540 low-side ON.
 * CH4 PWM B rising edge triggers on the up-counting half-cycle.
 * ADC sequence: Ia, Ib, Ic, temperature. Budget the ENTIRE sequence.
 */
#define AT32_FOC_TIMER_CLOCK_HZ       180000000u
#define AT32_FOC_TIMER_DIV_REG        1u
#define AT32_FOC_TIMER_ARR            2249u
#define AT32_FOC_TIMER_TICK_HZ        (AT32_FOC_TIMER_CLOCK_HZ / (AT32_FOC_TIMER_DIV_REG + 1u))
#define AT32_FOC_ADC_CLOCK_DIV        6u
#define AT32_FOC_ADC_CLOCK_HZ         (AT32_FOC_TIMER_CLOCK_HZ / AT32_FOC_ADC_CLOCK_DIV)
#define AT32_FOC_ADC_SEQUENCE_LENGTH  4u
#define AT32_FOC_ADC_SAMPLE_CYCLES_X2 27u /* ADC_SAMPLETIME_13_5 */
#define AT32_FOC_ADC_CONVERT_CYCLES_X2 25u /* 12-bit SAR: 12.5 cycles */
#define AT32_FOC_ADC_TRIGGER_CYCLES   3u /* trigger latency plus synchronization allowance */

/* Provisional board budget: MP6540 switching/sense recovery + external RC.
 * Must be qualified on the actual PCB over current, voltage and temperature.
 * The same margin is reserved before and after the complete ADC sequence.
 */
#ifndef AT32_FOC_CURRENT_SETTLING_NS
#define AT32_FOC_CURRENT_SETTLING_NS  2000u
#endif

#define AT32_FOC_CEIL_DIV(n, d) (((n) + (d) - 1u) / (d))
#define AT32_FOC_ADC_SEQUENCE_CYCLES_X2 \
  (AT32_FOC_ADC_SEQUENCE_LENGTH * (AT32_FOC_ADC_SAMPLE_CYCLES_X2 + AT32_FOC_ADC_CONVERT_CYCLES_X2) \
   + 2u * AT32_FOC_ADC_TRIGGER_CYCLES)
/* Clock ratio is exactly 3 on this board; use 64-bit integer arithmetic
 * so changes to the guard cannot silently overflow the nanosecond product. */
#define AT32_FOC_ADC_SEQUENCE_TICKS \
  AT32_FOC_CEIL_DIV(AT32_FOC_ADC_SEQUENCE_CYCLES_X2 * 1ULL * AT32_FOC_TIMER_TICK_HZ, \
                    2ULL * AT32_FOC_ADC_CLOCK_HZ)
#define AT32_FOC_SETTLING_TICKS \
  AT32_FOC_CEIL_DIV(AT32_FOC_CURRENT_SETTLING_NS * 1ULL * AT32_FOC_TIMER_TICK_HZ, 1000000000ULL)
#define AT32_FOC_ADC_TRIGGER_CCR \
  (AT32_FOC_TIMER_ARR - AT32_FOC_CEIL_DIV(AT32_FOC_ADC_SEQUENCE_TICKS, 2u))
#define AT32_FOC_CURRENT_MAX_CCR (AT32_FOC_ADC_TRIGGER_CCR - AT32_FOC_SETTLING_TICKS)
#define AT32_FOC_CURRENT_MAX_DUTY ((float)AT32_FOC_CURRENT_MAX_CCR / (float)AT32_FOC_TIMER_ARR)

#if AT32_FOC_ADC_SEQUENCE_TICKS >= (2u * AT32_FOC_TIMER_ARR)
#error "Injected ADC sequence does not fit in one PWM period"
#endif
#if (AT32_FOC_SETTLING_TICKS + AT32_FOC_CEIL_DIV(AT32_FOC_ADC_SEQUENCE_TICKS, 2u)) >= AT32_FOC_TIMER_ARR
#error "Current sampling window leaves no usable PWM duty range"
#endif
#if (2u * AT32_FOC_TIMER_ARR - AT32_FOC_ADC_TRIGGER_CCR - AT32_FOC_ADC_SEQUENCE_TICKS) < (AT32_FOC_CURRENT_MAX_CCR + AT32_FOC_SETTLING_TICKS)
#error "ADC sequence extends past the guaranteed low-side window"
#endif

#endif
