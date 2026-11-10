#include <stdint.h>
uint32_t dwt_get_ticks_us(void);
uint32_t dwt_get_ticks(void);
uint32_t dwt_elapsed_us(uint32_t start_ticks, uint32_t end_ticks);
uint32_t dwt_ticks_to_us(uint32_t elapsed_ticks);
