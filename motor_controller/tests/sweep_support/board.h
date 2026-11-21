#ifndef SWEEP_TEST_BOARD_H
#define SWEEP_TEST_BOARD_H
#include <stdint.h>
#include "at32_foc_timing.h"
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void __DMB(void);
#endif
