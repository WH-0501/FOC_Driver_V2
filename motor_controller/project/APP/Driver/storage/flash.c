#include "flash.h"
#include "at32m412_416_flash.h"
#include <stdint.h>
#include <string.h>

#ifndef FLASH_SECTOR_SIZE
#define FLASH_SECTOR_SIZE (0x800u) /* 2KB */
#endif

static uint32_t align_down(uint32_t v, uint32_t a)
{
  return v & ~(a - 1u);
}

static uint32_t align_up(uint32_t v, uint32_t a)
{
  return (v + (a - 1u)) & ~(a - 1u);
}

bool flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  if ((buf == NULL) || (len == 0u))
  {
    return false;
  }

  memcpy(buf, (const void *)addr, len);
  return true;
}

bool flash_erase(uint32_t addr, uint32_t len)
{
  uint32_t sector_addr;
  uint32_t end_addr;

  if (len == 0u)
  {
    return false;
  }

  sector_addr = align_down(addr, FLASH_SECTOR_SIZE);
  end_addr = align_up(addr + len, FLASH_SECTOR_SIZE);

  flash_unlock();
  while (sector_addr < end_addr)
  {
    if (flash_sector_erase(sector_addr) != FLASH_OPERATE_DONE)
    {
      flash_lock();
      return false;
    }
    sector_addr += FLASH_SECTOR_SIZE;
  }
  flash_lock();
  return true;
}

bool flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
  uint32_t i = 0u;
  uint16_t halfword_data;

  if ((buf == NULL) || (len == 0u))
  {
    return false;
  }
  if ((addr & 0x1u) != 0u)
  {
    /* halfword 编程要求 2 字节对齐地址。 */
    return false;
  }

  flash_unlock();

  /* 按 16-bit 写入；奇数字节尾部补 0xFF 后写半字。 */
  while ((i + 2u) <= len)
  {
    memcpy(&halfword_data, &buf[i], sizeof(uint16_t));
    if (flash_halfword_program(addr + i, halfword_data) != FLASH_OPERATE_DONE)
    {
      flash_lock();
      return false;
    }
    i += 2u;
  }

  while (i < len)
  {
    halfword_data = (uint16_t)buf[i] | 0xFF00u;
    if (flash_halfword_program(addr + i, halfword_data) != FLASH_OPERATE_DONE)
    {
      flash_lock();
      return false;
    }
    i += 1u;
  }

  flash_lock();
  return true;
}
