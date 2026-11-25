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
  /* Same CAN-FD DLC table, constant-time including the 64-byte stream path. */
  if (len <= 8u) return (can_data_length_type)len;
  if (len <= 24u) return (can_data_length_type)(((len + 3u) >> 2) + 6u);
  return (can_data_length_type)(len <= 32u ? 13u : len <= 48u ? 14u : 15u);
}

static void can_port_frame_from_rxbuf(const can_rxbuf_type *rx, comm_frame_t *out)
{
  if ((rx == NULL) || (out == NULL))
  {
    return;
  }

  out->id = rx->id;
  out->len = can_port_dlc_to_bytes(rx->data_length);
  out->bulk = false;
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

/* Foreground only. Revisit pending commits even after the last software slot
 * has been released: TSNEXT may still be high immediately after enqueue. */
static void can_port_at32_kick_bulk(void)
{
  if (!CAN1->ctrlstat_bit.tsnext && !CAN1->ctrlstat_bit.tsall &&
      !CAN1->ctrlstat_bit.tsone && !CAN1->ctrlstat_bit.tsa &&
      CAN1->ctrlstat_bit.tsstat != CAN_STB_STATUS_EMPTY)
    (void)can_txbuf_transmit(CAN1, CAN_TRANSMIT_STB_ALL);
}
static bool can_port_at32_tx_ready(void *ctx, bool bulk)
{
  (void)ctx;
  if (CAN1->ctrlstat_bit.stby) return false;
  if (bulk) return !CAN1->ctrlstat_bit.tsa && !CAN1->ctrlstat_bit.tsnext &&
                   CAN1->ctrlstat_bit.tsstat != CAN_STB_STATUS_FULL;
  return !CAN1->ctrlstat_bit.tpe && !CAN1->ctrlstat_bit.tpa;
}

/* This port sends standard data frames only. Follow the vendor TBSEL/TSNEXT
 * byte-write sequence, but copy the already serialized bytes directly to CAN
 * RAM. No temporary can_txbuf_type, full-struct memset, or byte repacking.
 * memcpy into a word is defined for unaligned sources (no aliasing casts).
 * Tail/DLC padding is explicitly zero; no read beyond frame->len. */
static void can_port_at32_write(const comm_frame_t *frame)
{
  uint8_t len = frame->len;
  can_data_length_type dlc = can_port_bytes_to_dlc(len);
  uint8_t padded = can_port_dlc_to_bytes(dlc);
  volatile uint8_t *reg = (volatile uint8_t *)&CAN1->ctrlstat + 1;
  const uint8_t *data = frame->data;
  volatile uint32_t *dest = CAN1->tbdat;
  unsigned words = len >> 2;
  uint32_t format = (uint32_t)dlc;
  *reg = (uint8_t)((*reg & 0x60u) | (frame->bulk ? 0x80u : 0u));
  CAN1->tbid = (frame->id & 0x7ffu) << 18;
#ifdef SUPPORT_CAN_FD
  format |= (1u << 17) | (1u << 18); /* FDF + BRS */
#endif
  CAN1->tbfmt = format;
  while (words--) {
    uint32_t word;
    memcpy(&word, data, 4); *dest++ = word; data += 4;
  }
  if (len & 3u) {
    uint32_t tail = 0u;
    memcpy(&tail, data, len & 3u); *dest++ = tail;
  }
  while (dest < CAN1->tbdat + ((padded + 3u) >> 2)) *dest++ = 0u;
  CAN1->tbtyp = 0u;
  CAN1->reserved8 = 0u;
  if (frame->bulk) {
    reg = (volatile uint8_t *)&CAN1->ctrlstat + 2;
    *reg = (uint8_t)(*reg | 0x40u); /* Commit once; hardware owns the slot. */
  }
}
static bool can_port_at32_send(void *ctx, const comm_frame_t *frame)
{
  if (!frame || frame->len > COMM_FRAME_MAX_LEN ||
      (frame->len && !frame->data) || !can_port_at32_tx_ready(ctx, frame->bulk))
    return false;
  can_port_at32_write(frame);
  if (frame->bulk)
  {
    /* A pending TSALL/TSNEXT is NOT a failed enqueue. poll_rx retries the kick. */
    can_port_at32_kick_bulk();
    return true;
  }
  return can_txbuf_transmit(CAN1, CAN_TRANSMIT_PTB) == SUCCESS;
}

static unsigned int can_port_at32_poll_rx(void *ctx,
                                          bool (*handler)(const comm_frame_t *frame, void *user),
                                          void *user)
{
  unsigned int handled = 0u;
  comm_frame_t frame;

  (void)ctx;

  can_port_at32_kick_bulk();
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
    frame.bulk = false;
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
    .tx_ready = can_port_at32_tx_ready,
    .rx_isr = can_port_at32_rx_irq_handler,
    .poll_rx = can_port_at32_poll_rx,
    .set_rx_filter = can_port_at32_set_rx_filter,
};

const comm_transport_ops_t *can_port_at32_ops(void)
{
  return &s_can_port_at32_ops;
}
