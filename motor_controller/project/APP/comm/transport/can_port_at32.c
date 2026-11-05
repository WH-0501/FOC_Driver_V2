#include "can_port_at32.h"
#include "comm.h"
#include "at32m412_416.h"
#include <stdbool.h>
#include <string.h>

#define CAN_PORT_RX_DMB() __DMB()
/* One hardware FIFO worth of work per interrupt; leave PWM IRQ latency bounded. */
#define CAN_PORT_RX_IRQ_BUDGET 6u

static can_rxbuf_type s_rx_pool[COMM_RX_QUEUE_DEPTH_MAX];
static uint8_t s_rx_depth = COMM_RX_QUEUE_DEPTH_DEFAULT;
static uint8_t s_rx_scratch[COMM_FRAME_MAX_LEN];

static struct
{
  volatile uint8_t head;
  volatile uint8_t tail;
  volatile uint32_t drop_count;
} s_rx_q;

static uint8_t can_port_dlc_to_bytes(can_data_length_type dlc)
{
  static const uint8_t table[16] = {
      0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
  };

  if ((uint32_t)dlc >= 16u)
  {
    return 0u;
  }

  return table[dlc];
}

static can_data_length_type can_port_bytes_to_dlc(uint8_t len)
{
  static const uint8_t lengths[16] = {
      0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
  };
  uint8_t i;

  for (i = 0u; i < 16u; ++i)
  {
    if (len <= lengths[i])
    {
      return (can_data_length_type)i;
    }
  }

  return CAN_DLC_BYTES_64;
}

static void can_port_frame_from_rxbuf(const can_rxbuf_type *rx, comm_frame_t *out)
{
  if ((rx == NULL) || (out == NULL))
  {
    return;
  }

  out->id = rx->id;
  out->len = can_port_dlc_to_bytes(rx->data_length);
  out->data = rx->data;
}

static can_rxbuf_type *can_port_rx_acquire_slot(void)
{
  uint8_t head;
  uint8_t next;

  head = s_rx_q.head;
  next = (uint8_t)((head + 1u) % s_rx_depth);
  if (next == s_rx_q.tail)
  {
    s_rx_q.drop_count++;
    return NULL;
  }

  return &s_rx_pool[head];
}

static void can_port_rx_commit_slot(void)
{
  CAN_PORT_RX_DMB();
  s_rx_q.head = (uint8_t)((s_rx_q.head + 1u) % s_rx_depth);
}

static void can_port_at32_rx_irq_handler(void)
{
  can_rxbuf_type *slot;
  can_rxbuf_type discard;
  uint8_t count;

  /* RIF is a sticky event, not a count of queued frames. Reading one frame and
   * clearing RIF leaves the rest waiting for the next bus frame indefinitely. */
  for (count = 0u; count < CAN_PORT_RX_IRQ_BUDGET; ++count)
  {
    if (can_rxbuf_status_get(CAN1) == CAN_RXBUF_STATUS_EMPTY)
    {
      break;
    }
    slot = can_port_rx_acquire_slot();
    /* Even with a full software ring, consume/release the hardware frame. */
    if (can_rxbuf_read(CAN1, slot != NULL ? slot : &discard) != SUCCESS)
    {
      break;
    }
    if (slot != NULL)
    {
      can_port_rx_commit_slot();
    }
  }

  /* Arrivals during draining may exhaust the budget. Resume without requiring
   * the host to send another command (including when hardware RIF is clear). */
  if (can_rxbuf_status_get(CAN1) != CAN_RXBUF_STATUS_EMPTY)
  {
    NVIC_SetPendingIRQ(CAN1_RX_IRQn);
  }
}

uint32_t can_port_at32_get_rx_drop_count(void)
{
  return s_rx_q.drop_count;
}

void can_port_at32_reset_rx_drop_count(void)
{
  s_rx_q.drop_count = 0u;
}

static void can_port_filter_exact_std(can_filter_config_type *filter, uint32_t id)
{
  can_filter_default_para_init(filter);

  filter->mask_para.id_type = FALSE;
  filter->code_para.id_type = CAN_ID_STANDARD;
  filter->mask_para.id = 0x000u;
  filter->code_para.id = id;
  filter->mask_para.data_length = 0xFu;
  filter->code_para.data_length = 0x0u;
  filter->mask_para.frame_type = TRUE;
  filter->code_para.frame_type = CAN_FRAME_DATA;
  filter->mask_para.recv_frame = TRUE;
  filter->code_para.recv_frame = CAN_RECV_NORMAL;
#ifdef SUPPORT_CAN_FD
  filter->mask_para.fd_format = TRUE;
  filter->code_para.fd_format = CAN_FORMAT_CLASSIC;
  filter->mask_para.fd_rate_switch = TRUE;
  filter->code_para.fd_rate_switch = CAN_BRS_OFF;
  filter->mask_para.fd_error_state = TRUE;
  filter->code_para.fd_error_state = CAN_ESI_ACTIVE;
#endif
}

static bool can_port_at32_apply_rx_filter(uint16_t node_id)
{
  can_filter_config_type filter;
  uint8_t i;

  if (node_id > 0x7FEu)
  {
    return false;
  }

  can_port_filter_exact_std(&filter, 0x000u);
  can_filter_set(CAN1, CAN_FILTER_NUM_0, &filter);

  can_port_filter_exact_std(&filter, (uint32_t)node_id);
  can_filter_set(CAN1, CAN_FILTER_NUM_1, &filter);

  can_port_filter_exact_std(&filter, 0x7FFu);
  can_filter_set(CAN1, CAN_FILTER_NUM_2, &filter);

  for (i = 3u; i < 16u; ++i)
  {
    can_filter_enable(CAN1, (can_filter_type)i, FALSE);
  }

  can_filter_enable(CAN1, CAN_FILTER_NUM_0, TRUE);
  can_filter_enable(CAN1, CAN_FILTER_NUM_1, TRUE);
  can_filter_enable(CAN1, CAN_FILTER_NUM_2, TRUE);

  return true;
}

static bool can_port_at32_init(void *ctx)
{
  const comm_config_t *cfg = (const comm_config_t *)ctx;
  uint8_t depth = COMM_RX_QUEUE_DEPTH_DEFAULT;

  if ((cfg != NULL) && (cfg->rx_queue_depth >= 2u))
  {
    depth = cfg->rx_queue_depth;
  }

  if (depth > COMM_RX_QUEUE_DEPTH_MAX)
  {
    depth = COMM_RX_QUEUE_DEPTH_MAX;
  }

  s_rx_depth = depth;
  s_rx_q.head = 0u;
  s_rx_q.tail = 0u;
  s_rx_q.drop_count = 0u;

  if ((cfg != NULL) && !can_port_at32_apply_rx_filter(cfg->node_id))
  {
    return false;
  }

  return true;
}

static bool can_port_at32_set_rx_filter(void *ctx, uint16_t node_id)
{
  (void)ctx;
  return can_port_at32_apply_rx_filter(node_id);
}

static void can_port_at32_deinit(void *ctx)
{
  (void)ctx;
  s_rx_q.head = 0u;
  s_rx_q.tail = 0u;
  s_rx_q.drop_count = 0u;
}

static bool can_port_at32_send(void *ctx, const comm_frame_t *frame)
{
  can_txbuf_type tx;
  uint8_t len;
  uint8_t i;

  (void)ctx;

  if (frame == NULL)
  {
    return false;
  }

  len = frame->len;
  if (len > COMM_FRAME_MAX_LEN)
  {
    len = COMM_FRAME_MAX_LEN;
  }

  memset(&tx, 0, sizeof(tx));
  tx.id = frame->id;
  tx.id_type = CAN_ID_STANDARD;
  tx.frame_type = CAN_FRAME_DATA;
  tx.data_length = can_port_bytes_to_dlc(len);
#ifdef SUPPORT_CAN_FD
  tx.fd_format = CAN_FORMAT_FD;
  tx.fd_rate_switch = CAN_BRS_ON;
#endif
  for (i = 0u; i < len; ++i)
  {
    tx.data[i] = frame->data[i];
  }

  if (can_txbuf_write(CAN1, CAN_TXBUF_PTB, &tx) != SUCCESS)
  {
    return false;
  }

  if (can_txbuf_transmit(CAN1, CAN_TRANSMIT_PTB) != SUCCESS)
  {
    return false;
  }

  return true;
}

static unsigned int can_port_at32_poll_rx(void *ctx,
                                          bool (*handler)(const comm_frame_t *frame, void *user),
                                          void *user)
{
  unsigned int handled = 0u;
  comm_frame_t frame;

  (void)ctx;

  while (s_rx_q.tail != s_rx_q.head)
  {
    const can_rxbuf_type *rx = &s_rx_pool[s_rx_q.tail];
    uint8_t len;

    len = can_port_dlc_to_bytes(rx->data_length);
    if (len > COMM_FRAME_MAX_LEN)
    {
      len = COMM_FRAME_MAX_LEN;
    }
    memcpy(s_rx_scratch, rx->data, len);

    frame.id = rx->id;
    frame.len = len;
    frame.data = s_rx_scratch;
    if (handler != NULL)
    {
      (void)handler(&frame, user);
    }

    CAN_PORT_RX_DMB();
    s_rx_q.tail = (uint8_t)((s_rx_q.tail + 1u) % s_rx_depth);
    handled++;
  }

  return handled;
}

static const comm_transport_ops_t s_can_port_at32_ops = {
    .name = "can_at32",
    .init = can_port_at32_init,
    .deinit = can_port_at32_deinit,
    .send = can_port_at32_send,
    .rx_isr = can_port_at32_rx_irq_handler,
    .poll_rx = can_port_at32_poll_rx,
    .set_rx_filter = can_port_at32_set_rx_filter,
};

const comm_transport_ops_t *can_port_at32_ops(void)
{
  return &s_can_port_at32_ops;
}
