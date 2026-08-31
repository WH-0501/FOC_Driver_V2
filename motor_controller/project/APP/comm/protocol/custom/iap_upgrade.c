#include "iap_upgrade.h"
#include "storage/flash.h"
#include "at32m412_416_misc.h"
#include <string.h>

#ifndef CAN_FD_FW_VERSION_STR
#define CAN_FD_FW_VERSION_STR "V1.0.0_FOC"
#endif

typedef struct __attribute__((packed))
{
  uint8_t signature[4];
  uint16_t image_type;
  uint16_t compress;
  uint32_t content_offset;
  uint32_t file_len_be;
  uint32_t reserved0;
  uint32_t checksum32;
  uint32_t reserved1[2];
} iap_staging_header_t;

typedef struct
{
  bool active;
  bool receiving;
  bool verified;
  uint32_t body_size;
  uint32_t checksum32;
  uint32_t bytes_written;
} iap_session_t;

static iap_session_t s_iap;

static uint32_t iap_align_up(uint32_t v, uint32_t a)
{
  return (v + (a - 1u)) & ~(a - 1u);
}

static uint32_t iap_read_be32(const uint8_t *p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static void iap_write_be32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

static uint32_t iap_firmware_checksum32(uint32_t body_addr, uint32_t body_len)
{
  uint32_t i;
  uint32_t count = body_len >> 2;
  uint32_t cksum = 0u;
  uint32_t data32;

  for (i = 0u; i < count; ++i)
  {
    data32 = *(const uint32_t *)(body_addr + (i * 4u));
    cksum += data32;
    if (cksum < data32)
    {
      cksum++;
    }
  }

  if ((body_len & 3u) != 0u)
  {
    data32 = *(const uint32_t *)(body_addr + (count * 4u));
    count = body_len & 3u;

    if (count == 3u)
    {
      data32 &= 0x00FFFFFFu;
    }
    else if (count == 2u)
    {
      data32 &= 0x0000FFFFu;
    }
    else
    {
      data32 &= 0x000000FFu;
    }

    cksum += data32;
    if (cksum < data32)
    {
      cksum++;
    }
  }

  return cksum;
}

static bool iap_erase_staging(uint32_t body_size)
{
  uint32_t total = IAP_STAGING_HEADER_SIZE + body_size;
  uint32_t erase_len = iap_align_up(total, IAP_FLASH_SECTOR_SIZE);

  if (erase_len > IAP_STAGING_REGION_SIZE)
  {
    return false;
  }

  return flash_erase(IAP_STAGING_BASE_ADDR, erase_len);
}

static bool iap_write_body_chunk(const uint8_t *data, uint32_t len)
{
  uint32_t addr;
  uint32_t space_left;

  if ((data == NULL) || (len == 0u))
  {
    return true;
  }

  if (!s_iap.receiving)
  {
    return false;
  }

  space_left = s_iap.body_size - s_iap.bytes_written;
  if (len > space_left)
  {
    len = space_left;
  }

  if (len == 0u)
  {
    return true;
  }

  addr = IAP_STAGING_BODY_ADDR + s_iap.bytes_written;
  if (!flash_write(addr, data, len))
  {
    return false;
  }

  s_iap.bytes_written += len;
  return true;
}

static void iap_set_result(iap_status_code_t *status, iap_error_code_t *err, iap_status_code_t st,
                           iap_error_code_t er)
{
  if (status != NULL)
  {
    *status = st;
  }
  if (err != NULL)
  {
    *err = er;
  }
}

bool iap_upgrade_is_active(void)
{
  return s_iap.active;
}

void iap_upgrade_reset_session(void)
{
  memset(&s_iap, 0, sizeof(s_iap));
}

bool iap_upgrade_enter(void)
{
  iap_upgrade_reset_session();
  s_iap.active = true;
  return true;
}

bool iap_upgrade_start(const uint8_t *payload, uint8_t payload_len, iap_status_code_t *status,
                       iap_error_code_t *err)
{
  uint32_t body_size;
  uint32_t checksum32;
  uint32_t data_len;

  if (!s_iap.active)
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_STATE);
    return false;
  }

  if (s_iap.receiving)
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_BUSY);
    return false;
  }

  if ((payload == NULL) || (payload_len <= 8u))
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_SIZE);
    return false;
  }

  body_size = iap_read_be32(&payload[0]);
  checksum32 = iap_read_be32(&payload[4]);

  if ((body_size == 0u) || (body_size > IAP_MAX_BODY_SIZE))
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_SIZE);
    return false;
  }

  if (!iap_erase_staging(body_size))
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_ERASE);
    return false;
  }

  s_iap.body_size = body_size;
  s_iap.checksum32 = checksum32;
  s_iap.bytes_written = 0u;
  s_iap.receiving = true;
  s_iap.verified = false;

  data_len = (uint32_t)payload_len - 8u;
  if (!iap_write_body_chunk(&payload[8], data_len))
  {
    s_iap.receiving = false;
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_WRITE);
    return false;
  }

  iap_set_result(status, err, IAP_STATUS_READY, IAP_ERR_NONE);
  return true;
}

bool iap_upgrade_data(const uint8_t *payload, uint8_t payload_len)
{
  if (!s_iap.active || !s_iap.receiving)
  {
    return false;
  }

  if ((payload == NULL) || (payload_len == 0u))
  {
    return true;
  }

  return iap_write_body_chunk(payload, payload_len);
}

bool iap_upgrade_end(iap_status_code_t *status, iap_error_code_t *err)
{
  if (!s_iap.active || !s_iap.receiving)
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_STATE);
    return false;
  }

  if (s_iap.bytes_written != s_iap.body_size)
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_INCOMPLETE);
    return false;
  }

  s_iap.receiving = false;
  iap_set_result(status, err, IAP_STATUS_READY, IAP_ERR_NONE);
  return true;
}

bool iap_upgrade_verify(iap_status_code_t *status, iap_error_code_t *err)
{
  iap_staging_header_t header;
  uint32_t computed;
  uint32_t file_len;
  uint8_t raw_header[sizeof(iap_staging_header_t)];

  if (!s_iap.active)
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_STATE);
    return false;
  }

  if (s_iap.receiving)
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_BUSY);
    return false;
  }

  if (s_iap.bytes_written != s_iap.body_size)
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_INCOMPLETE);
    return false;
  }

  computed = iap_firmware_checksum32(IAP_STAGING_BODY_ADDR, s_iap.body_size);
  if (computed != (~s_iap.checksum32))
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_CRC);
    return false;
  }

  memset(&header, 0, sizeof(header));
  header.signature[0] = 'A';
  header.signature[1] = 'S';
  header.signature[2] = 'I';
  header.signature[3] = 'X';
  header.image_type = 3u;
  header.compress = 0u;
  header.content_offset = 0u;
  file_len = IAP_STAGING_HEADER_SIZE + s_iap.body_size;
  iap_write_be32((uint8_t *)&header.file_len_be, file_len);
  header.checksum32 = s_iap.checksum32;

  memcpy(raw_header, &header, sizeof(raw_header));
  if (!flash_write(IAP_STAGING_BASE_ADDR, raw_header, sizeof(raw_header)))
  {
    iap_set_result(status, err, IAP_STATUS_ERROR, IAP_ERR_WRITE);
    return false;
  }

  s_iap.verified = true;
  iap_set_result(status, err, IAP_STATUS_VERIFIED, IAP_ERR_NONE);
  return true;
}

void iap_upgrade_reset_mcu(void)
{
  if (!s_iap.active)
  {
    return;
  }

  nvic_system_reset();
}

bool iap_upgrade_get_fw_version(uint8_t *buf, uint8_t buf_size, uint8_t *out_len)
{
  uint8_t ver_len;
  uint8_t i;

  if ((buf == NULL) || (buf_size == 0u))
  {
    return false;
  }

  ver_len = (uint8_t)strlen(CAN_FD_FW_VERSION_STR);
  if (ver_len >= buf_size)
  {
    ver_len = (uint8_t)(buf_size - 1u);
  }

  for (i = 0u; i < ver_len; ++i)
  {
    buf[i] = (uint8_t)CAN_FD_FW_VERSION_STR[i];
  }
  buf[ver_len] = 0u;

  if (out_len != NULL)
  {
    *out_len = (uint8_t)(ver_len + 1u);
  }

  return true;
}
