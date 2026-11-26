#include <assert.h>
#include <stdio.h>
static int adc_token;
#define ADC2 (&adc_token)
#define ADC_PCCE_FLAG 4u
#define RESET 0
static unsigned flag, calls, inject;
static int adc_interrupt_flag_get(void *adc, unsigned bit) { assert(adc==ADC2 && bit==ADC_PCCE_FLAG); return flag; }
static void adc_flag_clear(void *adc, unsigned bit) { assert(adc==ADC2 && bit==ADC_PCCE_FLAG); flag=0; }
static void callback(void *adc) { assert(adc==ADC2); ++calls; if(inject) flag=1; }
#define CURRENT_LOOP_IRQ_HANDLER callback
#include "adc_sequence_irq_impl.inc"
int main(void)
{
  ADC1_2_IRQHandler(); assert(calls==0);
  flag=1; ADC1_2_IRQHandler(); assert(calls==1 && flag==0);
  flag=1; inject=1; ADC1_2_IRQHandler(); assert(calls==2 && flag==1);
  inject=0; ADC1_2_IRQHandler(); assert(calls==3 && flag==0);
  puts("PASS: sequence IRQ acknowledges old event and preserves a newly completed conversion"); return 0;
}
