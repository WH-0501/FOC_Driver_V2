#include "comm_frame_queue.h"
#include <stddef.h>

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__)
#define COMM_FRAME_QUEUE_DMB() __asm volatile("dmb 0xF" :: : "memory")
#else
#define COMM_FRAME_QUEUE_DMB() do { } while (0)
#endif

void comm_frame_queue_init(comm_frame_queue_t *q, comm_frame_t *storage, uint8_t depth)
{
  if (q == NULL)
  {
    return;
  }

  q->frames = storage;
  q->depth = depth;
  q->head = 0u;
  q->tail = 0u;
  q->drop_count = 0u;
}

void comm_frame_queue_reset(comm_frame_queue_t *q)
{
  if (q == NULL)
  {
    return;
  }

  q->head = 0u;
  q->tail = 0u;
  q->drop_count = 0u;
}

uint8_t comm_frame_queue_capacity(const comm_frame_queue_t *q)
{
  if ((q == NULL) || (q->depth <= 1u))
  {
    return 0u;
  }

  return (uint8_t)(q->depth - 1u);
}

bool comm_frame_queue_push_isr(comm_frame_queue_t *q, const comm_frame_t *frame)
{
  uint8_t head;
  uint8_t next;

  if ((q == NULL) || (frame == NULL) || (q->frames == NULL) || (q->depth < 2u))
  {
    return false;
  }

  head = q->head;
  next = (uint8_t)((head + 1u) % q->depth);
  if (next == q->tail)
  {
    q->drop_count++;
    return false;
  }

  q->frames[head] = *frame;
  COMM_FRAME_QUEUE_DMB();
  q->head = next;
  return true;
}

bool comm_frame_queue_pop(comm_frame_queue_t *q, comm_frame_t *out)
{
  uint8_t tail;
  uint8_t head;

  if ((q == NULL) || (out == NULL) || (q->frames == NULL) || (q->depth < 2u))
  {
    return false;
  }

  tail = q->tail;
  head = q->head;
  if (tail == head)
  {
    return false;
  }

  *out = q->frames[tail];
  COMM_FRAME_QUEUE_DMB();
  q->tail = (uint8_t)((tail + 1u) % q->depth);
  return true;
}

unsigned int comm_frame_queue_drain(comm_frame_queue_t *q,
                                    bool (*handler)(const comm_frame_t *frame, void *user),
                                    void *user)
{
  unsigned int handled = 0u;
  comm_frame_t frame;

  if (q == NULL)
  {
    return 0u;
  }

  while (comm_frame_queue_pop(q, &frame))
  {
    if (handler != NULL)
    {
      (void)handler(&frame, user);
    }
    handled++;
  }

  return handled;
}

uint32_t comm_frame_queue_get_drop_count(const comm_frame_queue_t *q)
{
  if (q == NULL)
  {
    return 0u;
  }

  return q->drop_count;
}

void comm_frame_queue_reset_drop_count(comm_frame_queue_t *q)
{
  if (q == NULL)
  {
    return;
  }

  q->drop_count = 0u;
}
