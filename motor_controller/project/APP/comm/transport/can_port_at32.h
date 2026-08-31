#ifndef __CAN_PORT_AT32_H__
#define __CAN_PORT_AT32_H__

#include "at32m412_416_can.h"
#include "comm_transport.h"

/** 在 CAN1 RX 中断里调用：将硬件 RX 缓冲入队。 */
void can_port_at32_rx_push_isr(const can_rxbuf_type *rx);

/** RX 队列满时丢弃的帧计数（SPSC，可在调试/上位机诊断中读取）。 */
uint32_t can_port_at32_get_rx_drop_count(void);
void can_port_at32_reset_rx_drop_count(void);

const comm_transport_ops_t *can_port_at32_ops(void);

#endif /* __CAN_PORT_AT32_H__ */
