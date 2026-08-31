#ifndef __COMM_H__
#define __COMM_H__

#include <stdbool.h>
#include <stdint.h>
#include "at32m412_416_can.h"

typedef enum
{
  COMM_PROTOCOL_NONE = 0,
  COMM_PROTOCOL_CAN_FD_CUSTOM,
  COMM_PROTOCOL_CANOPEN,
} comm_protocol_type_t;

typedef struct
{
  comm_protocol_type_t protocol;
  /** 11-bit 标准 CAN Node-ID；为 0 时使用 motor_config.comm.can_id */
  uint16_t node_id;
} comm_config_t;

bool comm_init(const comm_config_t *cfg);
void comm_deinit(void);
void comm_poll(void);

/** 供 CAN1 RX 中断调用。 */
void comm_can1_rx_isr(const can_rxbuf_type *rx);

comm_protocol_type_t comm_get_active_protocol(void);

#endif /* __COMM_H__ */
