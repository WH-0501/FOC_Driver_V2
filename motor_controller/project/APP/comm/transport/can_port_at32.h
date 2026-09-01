#ifndef __CAN_PORT_AT32_H__
#define __CAN_PORT_AT32_H__

#include "comm_transport.h"

/** RX 队列满时丢弃的帧计数（SPSC，可在调试/上位机诊断中读取）。 */
uint32_t can_port_at32_get_rx_drop_count(void);
void can_port_at32_reset_rx_drop_count(void);

const comm_transport_ops_t *can_port_at32_ops(void);

#endif /* __CAN_PORT_AT32_H__ */
