#include "eeprom.h"
#include <stddef.h> 

#ifndef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE (16u)
#endif

static const eeprom_port_ops_t *s_port_ops = NULL;

void eeprom_bind_port(const eeprom_port_ops_t *ops)
{
  s_port_ops = ops;
}

bool eeprom_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  if ((s_port_ops == NULL) || (s_port_ops->read == NULL) || (buf == NULL) || (len == 0u))
  {
    return false;
  }

  return s_port_ops->read(addr, buf, len);
}

bool eeprom_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
  uint32_t offset = 0u;

  if ((s_port_ops == NULL) || (s_port_ops->write == NULL) || (buf == NULL) || (len == 0u))
  {
    return false;
  }

  /*
   * EEPROM 常见页写限制：这里按页切分，避免跨页写失败。
   * 底层若支持跨页，也可在 port->write 内自行处理。
   */
  while (offset < len)
  {
    uint32_t cur_addr = addr + offset;
    uint32_t page_remain = EEPROM_PAGE_SIZE - (cur_addr % EEPROM_PAGE_SIZE);
    uint32_t chunk = len - offset;
    if (chunk > page_remain)
    {
      chunk = page_remain;
    }

    if (!s_port_ops->write(cur_addr, &buf[offset], chunk))
    {
      return false;
    }
    offset += chunk;
  }

  return true;
}
