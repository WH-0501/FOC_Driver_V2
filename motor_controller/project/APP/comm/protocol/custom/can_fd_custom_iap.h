#ifndef __CAN_FD_CUSTOM_IAP_H__
#define __CAN_FD_CUSTOM_IAP_H__

#include <stdint.h>
#include <stdbool.h>
#include "comm_transport.h"

bool can_fd_iap_is_active(void);

void can_fd_iap_setup(bool (*send_frame)(uint32_t id, uint8_t cmd, const uint8_t *payload,
                                         uint8_t payload_len),
                      uint16_t *node_id, void (*cycle_set)(uint8_t enable));

void can_fd_iap_on_rx(const comm_frame_t *frame);

void can_fd_iap_poll(void);

#endif /* __CAN_FD_CUSTOM_IAP_H__ */
