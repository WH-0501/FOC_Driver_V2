#include "canopen.h"

static bool canopen_init(uint16_t node_id)
{
  (void)node_id;
  return false;
}

static void canopen_deinit(void)
{
}

static void canopen_on_rx(const comm_frame_t *frame)
{
  (void)frame;
}

static void canopen_poll(void)
{
}

static void canopen_bind_transport(bool (*send_fn)(const comm_frame_t *frame))
{
  (void)send_fn;
}

static const comm_protocol_ops_t s_canopen_ops = {
    .name = "canopen",
    .init = canopen_init,
    .deinit = canopen_deinit,
    .on_rx = canopen_on_rx,
    .poll = canopen_poll,
    .bind_transport = canopen_bind_transport,
};

const comm_protocol_ops_t *canopen_protocol_ops(void)
{
  return &s_canopen_ops;
}
