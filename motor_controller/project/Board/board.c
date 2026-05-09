#include "board.h"

void board_init(void)
{
    
}

void adc_irq_callback(void)
{
    // 读取ADC2的值
    uint16_t adc_value = adc_read(ADC2, ADC_CHANNEL_7);
}