#ifndef __COMM_TRANSPORT_H__
#define __COMM_TRANSPORT_H__

#include <stdbool.h>
#include <stdint.h>

#define COMM_FRAME_MAX_LEN 64u

/**
 * 通信帧：data 为外部缓冲指针，不含内嵌数组。
 * - RX：data 指向接收池，仅在 poll/on_rx 回调期间有效。
 * - TX：data 须指向发送方栈/静态缓冲，在 send 返回前保持有效。
 */
typedef struct
{
  uint32_t id;
  uint8_t len;
  const uint8_t *data;
} comm_frame_t;

typedef struct comm_transport_ops comm_transport_ops_t;

struct comm_transport_ops
{
  const char *name;
  bool (*init)(void *ctx);
  void (*deinit)(void *ctx);
  bool (*send)(void *ctx, const comm_frame_t *frame);
  /** RX 中断服务（平台 CAN 驱动在 RIF 等事件中调用 comm_can_rx_isr 间接进入）。 */
  void (*rx_isr)(void);
  unsigned int (*poll_rx)(void *ctx,
                          bool (*handler)(const comm_frame_t *frame, void *user),
                          void *user);
  /** 按 node_id 配置硬件 RX 过滤器；可为 NULL（无过滤能力的传输层）。 */
  bool (*set_rx_filter)(void *ctx, uint16_t node_id);
};

#endif /* __COMM_TRANSPORT_H__ */
