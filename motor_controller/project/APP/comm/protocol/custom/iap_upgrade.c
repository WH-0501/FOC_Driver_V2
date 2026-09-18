#include "iap_upgrade.h"
#include "at32m412_416_flash.h"
#include "at32m412_416_misc.h"
#include "dwt_profile_delay.h"
#include <string.h>

#define H2NL(x)                                                                                    \
  (((x) >> 24 & 0x000000FFu) | ((x) >> 8 & 0x0000FF00u) | ((x) << 24 & 0xFF000000u) |              \
   ((x) << 8 & 0x00FF0000u))

/* V / . / _ 为 ASCII，主/次/修订为数值字节 */
#define FOC_SOFT_VERSION_MAJOR 2u
#define FOC_SOFT_VERSION_MINOR 0u
#define FOC_SOFT_VERSION_PATCH 1u

#define BUILD_YEAR ((((uint32_t)__DATE__[9] - '0') * 10u) + ((uint32_t)__DATE__[10] - '0'))
#define BUILD_DAY                                                                                  \
  ((((uint32_t)(__DATE__[4] >= '0' ? __DATE__[4] - '0' : 0)) * 10u) +                              \
   ((uint32_t)__DATE__[5] - '0'))
#define BUILD_MONTH                                                                                \
  (__DATE__[2] == 'n'   ? (__DATE__[1] == 'a' ? 1u : 6u)                                           \
   : __DATE__[2] == 'b' ? 2u                                                                       \
   : __DATE__[2] == 'r' ? (__DATE__[0] == 'M' ? 3u : 4u)                                           \
   : __DATE__[2] == 'y' ? 5u                                                                       \
   : __DATE__[2] == 'l' ? 7u                                                                       \
   : __DATE__[2] == 'g' ? 8u                                                                       \
   : __DATE__[2] == 'p' ? 9u                                                                       \
   : __DATE__[2] == 't' ? 10u                                                                      \
   : __DATE__[2] == 'v' ? 11u                                                                      \
                        : 12u)
#define BUILD_HH ((((uint32_t)__TIME__[0] - '0') * 10u) + ((uint32_t)__TIME__[1] - '0'))
#define BUILD_MM ((((uint32_t)__TIME__[3] - '0') * 10u) + ((uint32_t)__TIME__[4] - '0'))
#define BUILD_SS ((((uint32_t)__TIME__[6] - '0') * 10u) + ((uint32_t)__TIME__[7] - '0'))

typedef struct
{
  IAP_Comm_t comm;
  IAP_Status_t status;
  uint32_t last_activity_us;
} IAP_Private_t;

typedef struct
{
  unsigned char Signature[4];
  unsigned short ImageType;
  unsigned short Compress;
  unsigned long ContentOffset;
  unsigned long FileLen;
  unsigned long Reserved0;
  unsigned long Checksum32;
  unsigned long Reserved1[2];
} UPGRADE_RT_HEAD;

typedef char iap_header_size_check[(sizeof(UPGRADE_RT_HEAD) == 32u) ? 1 : -1];

static IAP_Private_t iap_private;
static UPGRADE_RT_HEAD firmwareHeader;

static void iap_touch_activity(void)
{
  iap_private.last_activity_us = dwt_get_ticks_us();
  iap_private.comm.timeout_ms = 0u;
}

static uint32_t FirmwareChecksum32(uint32_t length)
{
  uint32_t i;
  uint32_t count = (length >> 2);
  uint32_t cksum = 0u;
  uint32_t data32;
  uint32_t firmwareAddr = IAP_STAGING_BASE_ADDR + sizeof(UPGRADE_RT_HEAD);

  for (i = 0u; i < count; i++)
  {
    data32 = *(volatile uint32_t *)firmwareAddr;
    cksum += data32;
    firmwareAddr += 4u;
    if (cksum < data32)
    {
      cksum++;
    }
  }

  if ((length % 4u) != 0u)
  {
    data32 = *(volatile uint32_t *)firmwareAddr;
    count = length % 4u;

    if (count == 3u)
    {
      data32 = (data32 & 0x00FFFFFFu);
    }
    else if (count == 2u)
    {
      data32 = (data32 & 0x0000FFFFu);
    }
    else
    {
      data32 = (data32 & 0x000000FFu);
    }

    cksum += data32;
    if (cksum < data32)
    {
      cksum++;
    }
  }

  return cksum;
}

static IAPError_t IAP_StartUpdate(void)
{
  uint32_t startAddress;
  uint8_t i;

  if ((firmwareHeader.Signature[0] != 'A') || (firmwareHeader.Signature[1] != 'S') ||
      (firmwareHeader.Signature[2] != 'I') || (firmwareHeader.Signature[3] != 'X'))
  {
    return ERROR_IAP_INVALID_PARAM;
  }

  if (firmwareHeader.ImageType != 0x3u)
  {
    return ERROR_IAP_INVALID_PARAM;
  }

  firmwareHeader.FileLen = H2NL(firmwareHeader.FileLen);
  if ((firmwareHeader.FileLen == 0u) ||
      (firmwareHeader.FileLen > (IAP_STAGING_MAX_SECTORS * IAP_FLASH_SECTOR_SIZE)))
  {
    return ERROR_IAP_INVALID_PARAM;
  }

  iap_private.status.total_size = firmwareHeader.FileLen;
  iap_private.status.current_size = 0u;
  iap_private.status.crc = firmwareHeader.Checksum32;
  iap_private.status.state = (uint8_t)IAP_STATE_RECEIVING;
  iap_private.status.current_earse = 0u;
  iap_private.status.earse_num =
      (uint8_t)((iap_private.status.total_size + (IAP_FLASH_SECTOR_SIZE - 1u)) / IAP_FLASH_SECTOR_SIZE);

  startAddress = IAP_STAGING_BASE_ADDR;
  flash_unlock();
  for (i = 0u; i < iap_private.status.earse_num; i++)
  {
    if (flash_sector_erase(startAddress) != FLASH_OPERATE_DONE)
    {
      flash_lock();
      return ERROR_IAP_FLASH_ERASE;
    }
    startAddress += IAP_FLASH_SECTOR_SIZE;
    iap_private.status.current_earse++;
  }
  flash_lock();

  return ERROR_IAP_SUCCESS;
}

static IAPError_t IAP_WriteFirmware(const uint8_t *data, uint8_t length, uint8_t data_following)
{
  static uint8_t iap_cache[4] = {0xFFu, 0xFFu, 0xFFu, 0xFFu};
  static uint8_t cache_count = 0u;
  static uint32_t startAddress = IAP_STAGING_BASE_ADDR;
  uint8_t i;

  if (iap_private.status.current_size == 0u)
  {
    memset(iap_cache, 0xFF, sizeof(iap_cache));
    cache_count = 0u;
    startAddress = IAP_STAGING_BASE_ADDR;
  }

  flash_unlock();

  for (i = 0u; i < length; i++)
  {
    if ((iap_private.status.current_size + cache_count) >= iap_private.status.total_size)
    {
      break;
    }

    iap_cache[cache_count++] = data[i];

    if (cache_count == 4u)
    {
      if (flash_word_program(startAddress, *(uint32_t *)iap_cache) != FLASH_OPERATE_DONE)
      {
        flash_lock();
        return ERROR_IAP_FLASH_WRITE;
      }
      startAddress += 4u;
      iap_private.status.current_size += 4u;
      memset(iap_cache, 0xFF, sizeof(iap_cache));
      cache_count = 0u;
    }
  }

  if ((data_following == 0u) && (cache_count != 0u))
  {
    if (flash_word_program(startAddress, *(uint32_t *)iap_cache) != FLASH_OPERATE_DONE)
    {
      flash_lock();
      return ERROR_IAP_FLASH_WRITE;
    }
    iap_private.status.current_size += cache_count;
    cache_count = 0u;
  }

  flash_lock();
  return ERROR_IAP_SUCCESS;
}

static IAPError_t IAP_VerifyFirmware(void)
{
  if (iap_private.status.current_size < iap_private.status.total_size)
  {
    return ERROR_IAP_LOST_FRAME;
  }

  if (FirmwareChecksum32(iap_private.status.total_size - sizeof(UPGRADE_RT_HEAD)) !=
      ~iap_private.status.crc)
  {
    return ERROR_IAP_FLASH_VERIFY;
  }

  return ERROR_IAP_SUCCESS;
}

IAP_Comm_t *IAP_Init(void)
{
  memset(&iap_private, 0, sizeof(iap_private));
  iap_private.status.state = (uint8_t)IAP_STATE_IDLE;
  iap_private.status.error = (uint8_t)ERROR_IAP_SUCCESS;
  iap_private.status.version[0] = (uint8_t)'V';
  iap_private.status.version[1] = (uint8_t)FOC_SOFT_VERSION_MAJOR;
  iap_private.status.version[2] = (uint8_t)'.';
  iap_private.status.version[3] = (uint8_t)FOC_SOFT_VERSION_MINOR;
  iap_private.status.version[4] = (uint8_t)'.';
  iap_private.status.version[5] = (uint8_t)FOC_SOFT_VERSION_PATCH;
  iap_private.status.version[6] = (uint8_t)'_';
  iap_private.status.build_time[0] = (uint8_t)BUILD_YEAR;
  iap_private.status.build_time[1] = (uint8_t)BUILD_MONTH;
  iap_private.status.build_time[2] = (uint8_t)BUILD_DAY;
  iap_private.status.build_time[3] = (uint8_t)BUILD_HH;
  iap_private.status.build_time[4] = (uint8_t)BUILD_MM;
  iap_private.status.build_time[5] = (uint8_t)BUILD_SS;
  iap_touch_activity();
  return &iap_private.comm;
}

void IAP_Process(const uint8_t *data, uint16_t length)
{
  uint8_t cmd;
  static uint8_t cnt;
  uint8_t buff[16];

  if ((data == NULL) || (length < 2u))
  {
    return;
  }

  cmd = data[1];
  iap_touch_activity();

  switch (cmd)
  {
    case IAP_CMD_ENTER:
      if (iap_private.status.state == (uint8_t)IAP_STATE_IDLE)
      {
        iap_private.status.state = (uint8_t)IAP_STATE_READY;
        if (iap_private.comm.motor != NULL)
        {
          iap_private.comm.motor(0u);
        }
        if (iap_private.comm.cycle != NULL)
        {
          iap_private.comm.cycle(0u);
        }
      }
      else
      {
        iap_private.status.error = (uint8_t)ERROR_IAP_SLAVE_REJECT;
      }
      break;

    case IAP_CMD_START:
      if ((iap_private.status.state == (uint8_t)IAP_STATE_READY) && (length >= 34u))
      {
        memcpy(&firmwareHeader, &data[2], sizeof(firmwareHeader));
        iap_private.status.error = (uint8_t)IAP_StartUpdate();
        if (iap_private.status.error != (uint8_t)ERROR_IAP_SUCCESS)
        {
          break;
        }
        buff[0] = (uint8_t)IAP_CMD_STATUS;
        buff[1] = iap_private.status.state;
        buff[2] = iap_private.status.error;
        if (iap_private.comm.send != NULL)
        {
          (void)iap_private.comm.send(buff, 3u);
        }
        cnt = data[0];
        iap_private.status.current_size = 0u;
        iap_private.status.error = (uint8_t)IAP_WriteFirmware(&data[2], (uint8_t)(length - 2u), 1u);
        iap_touch_activity();
      }
      else
      {
        iap_private.status.error = (uint8_t)ERROR_IAP_SLAVE_REJECT;
      }
      break;

    case IAP_CMD_DATA:
      cnt++;
      if (cnt != data[0])
      {
        iap_private.status.error = (uint8_t)ERROR_IAP_LOST_FRAME;
        break;
      }

      if (iap_private.status.state == (uint8_t)IAP_STATE_RECEIVING)
      {
        iap_private.status.error = (uint8_t)IAP_WriteFirmware(&data[2], (uint8_t)(length - 2u), 1u);
      }
      else
      {
        iap_private.status.error = (uint8_t)ERROR_IAP_SLAVE_REJECT;
      }
      break;

    case IAP_CMD_END:
      if (iap_private.status.state == (uint8_t)IAP_STATE_RECEIVING)
      {
        iap_private.status.error = (uint8_t)IAP_WriteFirmware(NULL, 0u, 0u);
        iap_private.status.state = (uint8_t)IAP_STATE_VERIFYING;
      }
      else
      {
        iap_private.status.error = (uint8_t)ERROR_IAP_BUSY;
      }
      break;

    case IAP_CMD_VERIFY:
      if (iap_private.status.state == (uint8_t)IAP_STATE_VERIFYING)
      {
        iap_private.status.error = (uint8_t)IAP_VerifyFirmware();
        iap_private.status.state = (uint8_t)IAP_STATE_COMPLETE;
      }
      else
      {
        iap_private.status.error = (uint8_t)ERROR_IAP_BUSY;
      }
      break;

    case IAP_CMD_RESET:
      nvic_system_reset();
      break;

    case IAP_CMD_STATUS:
      buff[0] = (uint8_t)IAP_CMD_STATUS;
      buff[1] = iap_private.status.state;
      buff[2] = iap_private.status.error;
      if (iap_private.comm.send != NULL)
      {
        (void)iap_private.comm.send(buff, 3u);
      }
      break;

    case IAP_CMD_VERSION:
      buff[0] = (uint8_t)IAP_CMD_VERSION;
      memcpy(&buff[1], iap_private.status.version, sizeof(iap_private.status.version));
      memcpy(&buff[1 + sizeof(iap_private.status.version)], iap_private.status.build_time,
             sizeof(iap_private.status.build_time));
      if (iap_private.comm.send != NULL)
      {
        (void)iap_private.comm.send(
            buff, (uint16_t)(1u + sizeof(iap_private.status.version) +
                             sizeof(iap_private.status.build_time)));
      }
      break;

    default:
      iap_private.status.error = (uint8_t)ERROR_IAP_INVALID_COMMAND;
      break;
  }
}

void IAP_Error(void)
{
  uint8_t buff[8];
  uint32_t now_us;
  uint32_t elapsed_ms;

  if (iap_private.status.state > (uint8_t)IAP_STATE_IDLE)
  {
    now_us = dwt_get_ticks_us();
    elapsed_ms = (now_us - iap_private.last_activity_us) / 1000u;
    iap_private.comm.timeout_ms = elapsed_ms;
    if (elapsed_ms > IAP_TIMEOUT_MS)
    {
      iap_private.status.error = (uint8_t)ERROR_IAP_TIMEOUT;
    }

    if (iap_private.status.error != (uint8_t)ERROR_IAP_SUCCESS)
    {
      if (iap_private.status.state >= (uint8_t)IAP_STATE_RECEIVING)
      {
        flash_unlock();
        (void)flash_sector_erase(IAP_STAGING_BASE_ADDR);
        flash_lock();
      }

      iap_private.status.state = (uint8_t)IAP_STATE_ERROR;
      buff[0] = (uint8_t)IAP_CMD_STATUS;
      buff[1] = iap_private.status.state;
      buff[2] = iap_private.status.error;
      if (iap_private.comm.send != NULL)
      {
        (void)iap_private.comm.send(buff, 3u);
      }

      iap_private.status.state = (uint8_t)IAP_STATE_IDLE;
      iap_private.status.error = (uint8_t)ERROR_IAP_SUCCESS;
      iap_touch_activity();
      if (iap_private.comm.cycle != NULL)
      {
        iap_private.comm.cycle(1u);
      }
    }
  }
}

IAP_Status_t *IAP_GetStatus(void)
{
  return &iap_private.status;
}

bool IAP_IsActive(void)
{
  return (iap_private.status.state > (uint8_t)IAP_STATE_IDLE);
}
