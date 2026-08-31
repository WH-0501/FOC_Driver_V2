#ifndef __CANOPEN_PROTOCOL_H__
#define __CANOPEN_PROTOCOL_H__

#include "comm_protocol.h"

/** CANopen 协议插件占位；后续在此实现 NMT/SDO/PDO。 */
const comm_protocol_ops_t *canopen_protocol_ops(void);

#endif /* __CANOPEN_PROTOCOL_H__ */
