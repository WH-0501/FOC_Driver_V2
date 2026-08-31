#ifndef __COMM_FRAME_QUEUE_H__
#define __COMM_FRAME_QUEUE_H__

#include <stdbool.h>
#include <stdint.h>
#include "comm_transport.h"

/**
 * 跨平台 SPSC 通信帧队列（ISR/线程 写，主循环 读）。
 * 存储区由调用方提供；有效容量为 depth - 1 帧。
 */
typedef struct
{
  comm_frame_t *frames;
  uint8_t depth;
  volatile uint8_t head;
  volatile uint8_t tail;
  volatile uint32_t drop_count;
} comm_frame_queue_t;

void comm_frame_queue_init(comm_frame_queue_t *q, comm_frame_t *storage, uint8_t depth);
void comm_frame_queue_reset(comm_frame_queue_t *q);

/** 生产者侧（通常在 RX 中断里调用）。满则丢弃并累加 drop_count。 */
bool comm_frame_queue_push_isr(comm_frame_queue_t *q, const comm_frame_t *frame);

/** 消费者侧：弹出单帧；空则返回 false。 */
bool comm_frame_queue_pop(comm_frame_queue_t *q, comm_frame_t *out);

/** 消费者侧：弹出并处理全部待读帧，返回处理数量。 */
unsigned int comm_frame_queue_drain(comm_frame_queue_t *q,
                                    bool (*handler)(const comm_frame_t *frame, void *user),
                                    void *user);

uint8_t comm_frame_queue_capacity(const comm_frame_queue_t *q);
uint32_t comm_frame_queue_get_drop_count(const comm_frame_queue_t *q);
void comm_frame_queue_reset_drop_count(comm_frame_queue_t *q);

#endif /* __COMM_FRAME_QUEUE_H__ */
