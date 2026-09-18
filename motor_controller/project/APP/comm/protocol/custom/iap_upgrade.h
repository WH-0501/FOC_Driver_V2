#ifndef __IAP_UPGRADE_H__
#define __IAP_UPGRADE_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * App 侧 IAP（对齐 dex-con-firmware-v5 motor_control/project/IAP）。
 * CAN 下载整包（32B ASIX 头 + bin）到 staging，复位后由 AT32M416_Bldr 拷贝到 APP。
 */
#ifndef IAP_APPLICATION_ADDRESS
#define IAP_APPLICATION_ADDRESS 0x08001000u
#endif

#ifndef IAP_STAGING_BASE_ADDR
#define IAP_STAGING_BASE_ADDR 0x08010400u
#endif

#ifndef IAP_FLASH_SECTOR_SIZE
#define IAP_FLASH_SECTOR_SIZE 1024u
#endif

#ifndef IAP_STAGING_MAX_SECTORS
#define IAP_STAGING_MAX_SECTORS 61u
#endif

#ifndef IAP_TIMEOUT_MS
#define IAP_TIMEOUT_MS 5000u
#endif

typedef enum
{
  IAP_STATE_IDLE = 0,
  IAP_STATE_READY,
  IAP_STATE_RECEIVING,
  IAP_STATE_VERIFYING,
  IAP_STATE_PROGRAMMING,
  IAP_STATE_COMPLETE,
  IAP_STATE_ERROR
} IAP_State_t;

typedef enum
{
  ERROR_IAP_SUCCESS = 0x80,
  ERROR_IAP_INVALID_COMMAND,
  ERROR_IAP_INVALID_PARAM,
  ERROR_IAP_FLASH_ERASE,
  ERROR_IAP_FLASH_WRITE,
  ERROR_IAP_FLASH_VERIFY,
  ERROR_IAP_CRC,
  ERROR_IAP_TIMEOUT,
  ERROR_IAP_BUSY,
  ERROR_IAP_NOT_SUPPORTED,
  ERROR_IAP_SLAVE_COMM,
  ERROR_IAP_SLAVE_TIMEOUT,
  ERROR_IAP_SLAVE_REJECT,
  ERROR_IAP_MALLOC_FAILED,
  ERROR_IAP_FLASH_SWITCH,
  ERROR_IAP_LOST_FRAME,
} IAPError_t;

typedef enum
{
  IAP_CMD_ENTER = 0xF8,
  IAP_CMD_START = 0xF9,
  IAP_CMD_DATA = 0xFA,
  IAP_CMD_END = 0xFB,
  IAP_CMD_STATUS = 0xFC,
  IAP_CMD_VERSION = 0xFD,
  IAP_CMD_VERIFY = 0xFE,
  IAP_CMD_RESET = 0xFF,
} IAP_Cmd_t;

typedef struct
{
  bool (*send)(uint8_t *data, uint16_t length);
  void (*motor)(uint8_t enable);
  void (*cycle)(uint8_t enable);
  uint32_t timeout_ms;
} IAP_Comm_t;

typedef struct
{
  uint8_t state;
  uint8_t error;
  uint8_t earse_num;
  uint8_t current_earse;
  uint8_t version[7];
  uint8_t build_time[6];
  uint32_t total_size;
  uint32_t current_size;
  uint32_t crc;
} IAP_Status_t;

IAP_Comm_t *IAP_Init(void);
void IAP_Process(const uint8_t *data, uint16_t length);
void IAP_Error(void);
IAP_Status_t *IAP_GetStatus(void);
bool IAP_IsActive(void);

#endif /* __IAP_UPGRADE_H__ */
