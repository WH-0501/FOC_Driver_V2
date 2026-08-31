#ifndef __CAN_FD_CUSTOM_IAP_H__
#define __CAN_FD_CUSTOM_IAP_H__

#include <stdint.h>
#include <stdbool.h>
#include "comm_transport.h"

bool can_fd_iap_is_active(void);

void can_fd_iap_on_rx(uint32_t frame_id, uint8_t cmd, const uint8_t *payload, uint8_t payload_len,
                      bool (*send_fn)(const comm_frame_t *frame), uint32_t reply_id, uint8_t *tx_cnt);

#endif /* __CAN_FD_CUSTOM_IAP_H__ */
