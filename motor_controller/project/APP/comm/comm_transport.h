#ifndef __COMM_TRANSPORT_H__
#define __COMM_TRANSPORT_H__

#include <stdbool.h>
#include <stdint.h>

/** 与具体总线无关的一帧数据（CAN / 未来 EtherCAT 等共用）。 */
typedef struct
{
  uint32_t id;
  uint8_t len;
  uint8_t data[64];
} comm_frame_t;

typedef struct comm_transport_ops comm_transport_ops_t;

struct comm_transport_ops
{
  const char *name;
  bool (*init)(void *ctx);
  void (*deinit)(void *ctx);
  bool (*send)(void *ctx, const comm_frame_t *frame);
  /** 主循环中调用：将 RX 队列中的帧交给上层，返回处理的帧数。 */
  unsigned int (*poll_rx)(void *ctx,
                          bool (*handler)(const comm_frame_t *frame, void *user),
                          void *user);
};

#endif /* __COMM_TRANSPORT_H__ */
