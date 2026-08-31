#ifndef __COMM_PROTOCOL_H__
#define __COMM_PROTOCOL_H__

#include <stdbool.h>
#include <stdint.h>
#include "comm_transport.h"

typedef struct comm_protocol_ops comm_protocol_ops_t;

struct comm_protocol_ops
{
  const char *name;
  bool (*init)(uint16_t node_id);
  void (*deinit)(void);
  /** 收到一帧（已由传输层解包为 comm_frame_t）。 */
  void (*on_rx)(const comm_frame_t *frame);
  /** 主循环周期任务：周期上报、通讯看门狗等。 */
  void (*poll)(void);
  /** 若需主动发应答，通过该 send 回调写入总线。 */
  void (*bind_transport)(bool (*send_fn)(const comm_frame_t *frame));
};

#endif /* __COMM_PROTOCOL_H__ */
