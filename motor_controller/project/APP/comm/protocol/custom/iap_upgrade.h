#ifndef __IAP_UPGRADE_H__
#define __IAP_UPGRADE_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * IAP 升级分区（与 AT32M416_Bldr/app/flash/flash.c 对齐）。
 * App 负责 CAN 下载至 staging，复位后 Bootloader 拷贝至 APPLICATION。
 */
#ifndef IAP_APPLICATION_ADDRESS
#define IAP_APPLICATION_ADDRESS     0x08001000u
#endif

#ifndef IAP_STAGING_BASE_ADDR
#define IAP_STAGING_BASE_ADDR       0x08010400u
#endif

#ifndef IAP_STAGING_HEADER_SIZE
#define IAP_STAGING_HEADER_SIZE     32u
#endif

#ifndef IAP_STAGING_BODY_ADDR
#define IAP_STAGING_BODY_ADDR       (IAP_STAGING_BASE_ADDR + IAP_STAGING_HEADER_SIZE)
#endif

#ifndef IAP_FLASH_SECTOR_SIZE
#define IAP_FLASH_SECTOR_SIZE       1024u
#endif

#ifndef IAP_STAGING_MAX_SECTORS
#define IAP_STAGING_MAX_SECTORS     61u
#endif

#define IAP_STAGING_REGION_SIZE     (IAP_STAGING_MAX_SECTORS * IAP_FLASH_SECTOR_SIZE)
#define IAP_MAX_BODY_SIZE           (IAP_STAGING_REGION_SIZE - IAP_STAGING_HEADER_SIZE)

/** IAP 状态（0xFC 应答 byte0）。 */
typedef enum
{
  IAP_STATUS_IDLE = 0,
  IAP_STATUS_READY = 1,
  IAP_STATUS_VERIFIED = 2,
  IAP_STATUS_ERROR = 3,
} iap_status_code_t;

/** IAP 错误码（0xFC 应答 byte1）。 */
typedef enum
{
  IAP_ERR_NONE = 0,
  IAP_ERR_BUSY = 1,
  IAP_ERR_SIZE = 2,
  IAP_ERR_ERASE = 3,
  IAP_ERR_WRITE = 4,
  IAP_ERR_CRC = 5,
  IAP_ERR_STATE = 6,
  IAP_ERR_OVERFLOW = 7,
  IAP_ERR_INCOMPLETE = 8,
} iap_error_code_t;

bool iap_upgrade_is_active(void);

void iap_upgrade_reset_session(void);

bool iap_upgrade_enter(void);

bool iap_upgrade_start(const uint8_t *payload, uint8_t payload_len, iap_status_code_t *status,
                       iap_error_code_t *err);

bool iap_upgrade_data(const uint8_t *payload, uint8_t payload_len);

bool iap_upgrade_end(iap_status_code_t *status, iap_error_code_t *err);

bool iap_upgrade_verify(iap_status_code_t *status, iap_error_code_t *err);

void iap_upgrade_reset_mcu(void);

bool iap_upgrade_get_fw_version(uint8_t *buf, uint8_t buf_size, uint8_t *out_len);

#endif /* __IAP_UPGRADE_H__ */
