#ifndef __FLASH_H__
#define __FLASH_H__

#include <stdbool.h>
#include <stdint.h>

bool flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
bool flash_erase(uint32_t addr, uint32_t len);
bool flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);

#endif /* __FLASH_H__ */
