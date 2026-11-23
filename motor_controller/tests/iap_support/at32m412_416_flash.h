#include <stdint.h>
#define FLASH_OPERATE_DONE 0
void flash_unlock(void);
void flash_lock(void);
int flash_sector_erase(uint32_t address);
int flash_word_program(uint32_t address, uint32_t value);
