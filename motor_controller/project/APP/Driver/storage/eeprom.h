#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool (*read)(uint32_t addr, uint8_t *buf, uint32_t len);
  bool (*write)(uint32_t addr, const uint8_t *buf, uint32_t len);
} eeprom_port_ops_t;

void eeprom_bind_port(const eeprom_port_ops_t *ops);

bool eeprom_read(uint32_t addr, uint8_t *buf, uint32_t len);
bool eeprom_write(uint32_t addr, const uint8_t *buf, uint32_t len);

#endif /* __EEPROM_H__ */
