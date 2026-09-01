#ifndef __COMM_H__
#define __COMM_H__

#include <stdbool.h>
#include <stdint.h>

/** 默认 RX 环形队列深度（槽位数；SPSC 有效容量 = depth - 1）。可在工程/board 头文件中覆盖。 */
#ifndef COMM_RX_QUEUE_DEPTH_DEFAULT
#define COMM_RX_QUEUE_DEPTH_DEFAULT 32u
#endif

/** 各平台 RX 池编译期上限（静态分配数组大小）。 */
#ifndef COMM_RX_QUEUE_DEPTH_MAX
#define COMM_RX_QUEUE_DEPTH_MAX 64u
#endif

#if COMM_RX_QUEUE_DEPTH_DEFAULT > COMM_RX_QUEUE_DEPTH_MAX
#error "COMM_RX_QUEUE_DEPTH_DEFAULT must not exceed COMM_RX_QUEUE_DEPTH_MAX"
#endif

#if COMM_RX_QUEUE_DEPTH_DEFAULT < 2u
#error "COMM_RX_QUEUE_DEPTH_DEFAULT must be >= 2"
#endif

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
  /**
   * RX 队列深度（槽位数）；0 表示使用 COMM_RX_QUEUE_DEPTH_DEFAULT。
   * 有效缓冲帧数 = depth - 1，须 >= 2 且 <= COMM_RX_QUEUE_DEPTH_MAX。
   */
  uint8_t rx_queue_depth;
} comm_config_t;

bool comm_init(const comm_config_t *cfg);
void comm_deinit(void);
void comm_poll(void);

/** CAN RX 中断入口：委托当前传输层完成收帧入队。 */
void comm_can_rx_isr(void);

comm_protocol_type_t comm_get_active_protocol(void);

/** 运行时更新 RX 硬件过滤器（如 SET_NODE_ID）；委托当前传输层。 */
bool comm_set_node_id_filter(uint16_t node_id);

#endif /* __COMM_H__ */
