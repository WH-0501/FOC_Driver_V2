#include "can_fd_custom_iap.h"
#include "can_fd_custom.h"
#include "can_fd_custom_codec.h"
#include "iap_upgrade.h"
#include "mc_interface.h"
#include "comm_transport.h"
#include <string.h>

static bool (*s_send_fn)(const comm_frame_t *frame);

static bool can_fd_iap_send(uint32_t id, uint8_t cmd, const uint8_t *payload, uint8_t payload_len,
                            uint8_t *tx_cnt)
{
  comm_frame_t frame;
  uint8_t buf[COMM_FRAME_MAX_LEN];
  uint8_t total_len;
  uint8_t padded_len;
  uint8_t i;

  if (s_send_fn == NULL)
  {
    return false;
  }

  total_len = (uint8_t)(2u + payload_len);
  padded_len = can_fd_codec_pad_canfd_len(total_len);
  if (padded_len > COMM_FRAME_MAX_LEN)
  {
    return false;
  }

  buf[0] = (*tx_cnt)++;
  buf[1] = cmd;
  for (i = 0u; i < payload_len; ++i)
  {
    buf[2u + i] = payload[i];
  }
  for (i = total_len; i < padded_len; ++i)
  {
    buf[i] = 0u;
  }

  frame.id = id;
  frame.len = padded_len;
  frame.data = buf;

  return s_send_fn(&frame);
}

static void can_fd_iap_reply_status(uint32_t reply_id, uint8_t *tx_cnt, iap_status_code_t status,
                                    iap_error_code_t err)
{
  uint8_t payload[2];

  payload[0] = (uint8_t)status;
  payload[1] = (uint8_t)err;
  (void)can_fd_iap_send(reply_id, CAN_FD_CMD_IAP_STATUS, payload, 2u, tx_cnt);
}

bool can_fd_iap_is_active(void)
{
  return iap_upgrade_is_active();
}

void can_fd_iap_on_rx(uint32_t frame_id, uint8_t cmd, const uint8_t *payload, uint8_t payload_len,
                      bool (*send_fn)(const comm_frame_t *frame), uint32_t reply_id, uint8_t *tx_cnt)
{
  iap_status_code_t status = IAP_STATUS_IDLE;
  iap_error_code_t err = IAP_ERR_NONE;
  uint8_t ver_buf[16];
  uint8_t ver_len = 0u;

  (void)frame_id;
  s_send_fn = send_fn;

  switch (cmd)
  {
    case CAN_FD_CMD_IAP_ENTER:
      (void)mc_disable();
      (void)iap_upgrade_enter();
      break;

    case CAN_FD_CMD_IAP_START:
      (void)iap_upgrade_start(payload, payload_len, &status, &err);
      can_fd_iap_reply_status(reply_id, tx_cnt, status, err);
      break;

    case CAN_FD_CMD_IAP_DATA:
      if (!iap_upgrade_data(payload, payload_len))
      {
        can_fd_iap_reply_status(reply_id, tx_cnt, IAP_STATUS_ERROR, IAP_ERR_WRITE);
      }
      break;

    case CAN_FD_CMD_IAP_END:
      if (!iap_upgrade_end(&status, &err))
      {
        can_fd_iap_reply_status(reply_id, tx_cnt, status, err);
      }
      break;

    case CAN_FD_CMD_IAP_VERIFY:
      (void)iap_upgrade_verify(&status, &err);
      can_fd_iap_reply_status(reply_id, tx_cnt, status, err);
      break;

    case CAN_FD_CMD_IAP_RESET:
      iap_upgrade_reset_mcu();
      break;

    case CAN_FD_CMD_IAP_FW_VERSION:
      if (iap_upgrade_get_fw_version(ver_buf, (uint8_t)sizeof(ver_buf), &ver_len))
      {
        (void)can_fd_iap_send(reply_id, CAN_FD_CMD_IAP_FW_VERSION, ver_buf, ver_len, tx_cnt);
      }
      break;

    case CAN_FD_CMD_IAP_STATUS:
      can_fd_iap_reply_status(reply_id, tx_cnt, IAP_STATUS_IDLE, IAP_ERR_NONE);
      break;

    default:
      break;
  }
}
