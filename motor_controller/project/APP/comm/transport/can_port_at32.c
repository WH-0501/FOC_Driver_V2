#include "can_port_at32.h"
#include "comm_frame_queue.h"
#include "at32m412_416.h"
#include <string.h>

/*
 * RX 队列深度（comm_frame_queue 有效容量 = DEPTH - 1）。
 * 链路：仲裁 1 Mbps，CAN-FD 数据段 5 Mbps；31 帧约 3~9 ms 突发余量。
 */
#ifndef CAN_PORT_AT32_RX_QUEUE_DEPTH
#define CAN_PORT_AT32_RX_QUEUE_DEPTH 32u
#endif

static comm_frame_t s_rx_storage[CAN_PORT_AT32_RX_QUEUE_DEPTH];
static comm_frame_queue_t s_rx_queue;

/**
 * @brief CAN-FD 数据长度转换为字节数
 * @param dlc CAN-FD 数据长度
 * @return 字节数
 */
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

/**
 * @brief 字节数转换为 CAN-FD 数据长度
 * @param len 字节数
 * @return CAN-FD 数据长度
 */
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

/**
 * @brief CAN-FD 接收缓冲区数据转换为 comm_frame_t  
 * @param rx CAN-FD 接收缓冲区数据
 * @param out 输出 comm_frame_t
 * @return true 成功, false 失败
 */ 
static bool can_port_frame_from_rxbuf(const can_rxbuf_type *rx, comm_frame_t *out)
{
  uint8_t len;

  if ((rx == NULL) || (out == NULL))
  {
    return false;
  }

  len = can_port_dlc_to_bytes(rx->data_length);
  if (len > (uint8_t)sizeof(out->data))
  {
    len = (uint8_t)sizeof(out->data);
  }

  out->id = rx->id;
  out->len = len;
  if (len > 0u)
  {
    memcpy(out->data, rx->data, len);
  }

  return true;
}

/**
 * @brief 将 CAN-FD 接收缓冲区数据转换为 comm_frame_t 并推入队列
 * @param rx CAN-FD 接收缓冲区数据
 */
void can_port_at32_rx_push_isr(const can_rxbuf_type *rx)
{
  comm_frame_t frame;

  if (!can_port_frame_from_rxbuf(rx, &frame))
  {
    return;
  }

  (void)comm_frame_queue_push_isr(&s_rx_queue, &frame);
}

uint32_t can_port_at32_get_rx_drop_count(void)
{
  return comm_frame_queue_get_drop_count(&s_rx_queue);
}

void can_port_at32_reset_rx_drop_count(void)
{
  comm_frame_queue_reset_drop_count(&s_rx_queue);
}

static bool can_port_at32_init(void *ctx)
{
  (void)ctx;
  comm_frame_queue_init(&s_rx_queue, s_rx_storage, (uint8_t)CAN_PORT_AT32_RX_QUEUE_DEPTH);
  return true;
}

static void can_port_at32_deinit(void *ctx)
{
  (void)ctx;
  comm_frame_queue_reset(&s_rx_queue);
}

static bool can_port_at32_send(void *ctx, const comm_frame_t *frame)
{
  can_txbuf_type tx;
  uint8_t len;

  (void)ctx;

  if (frame == NULL)
  {
    return false;
  }

  len = frame->len;
  if (len > 64u)
  {
    len = 64u;
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
  if (len > 0u)
  {
    memcpy(tx.data, frame->data, len);
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
  (void)ctx;
  return comm_frame_queue_drain(&s_rx_queue, handler, user);
}

static const comm_transport_ops_t s_can_port_at32_ops = {
    .name = "can_at32",
    .init = can_port_at32_init,
    .deinit = can_port_at32_deinit,
    .send = can_port_at32_send,
    .poll_rx = can_port_at32_poll_rx,
};

const comm_transport_ops_t *can_port_at32_ops(void)
{
  return &s_can_port_at32_ops;
}
