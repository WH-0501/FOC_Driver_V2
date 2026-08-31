#include "comm.h"
#include "comm_transport.h"
#include "comm_protocol.h"
#include "can_port_at32.h"
#include "can_fd_custom.h"
#include "canopen.h"
#include "motor.h"

static const comm_transport_ops_t *s_transport;
static const comm_protocol_ops_t *s_protocol;
static comm_protocol_type_t s_active_protocol = COMM_PROTOCOL_NONE;

static bool comm_send_wrapper(const comm_frame_t *frame)
{
  if ((s_transport == NULL) || (s_transport->send == NULL))
  {
    return false;
  }

  return s_transport->send(NULL, frame);
}

static bool comm_rx_dispatch(const comm_frame_t *frame, void *user)
{
  (void)user;

  if ((s_protocol != NULL) && (s_protocol->on_rx != NULL))
  {
    s_protocol->on_rx(frame);
  }

  return true;
}

static const comm_protocol_ops_t *comm_select_protocol_ops(comm_protocol_type_t type)
{
  switch (type)
  {
    case COMM_PROTOCOL_CAN_FD_CUSTOM:
      return can_fd_custom_protocol_ops();
    case COMM_PROTOCOL_CANOPEN:
      return canopen_protocol_ops();
    default:
      return NULL;
  }
}

bool comm_init(const comm_config_t *cfg)
{
  uint16_t node_id;

  if (cfg == NULL)
  {
    return false;
  }

  s_transport = can_port_at32_ops();
  s_protocol = comm_select_protocol_ops(cfg->protocol);
  if (s_protocol == NULL)
  {
    return false;
  }

  if ((s_transport->init != NULL) && !s_transport->init(NULL))
  {
    return false;
  }

  node_id = cfg->node_id;
  if (node_id == 0u)
  {
    node_id = motor_get_handler()->config.comm.can_id;
  }

  if ((s_protocol->init != NULL) && !s_protocol->init(node_id))
  {
    return false;
  }

  if (s_protocol->bind_transport != NULL)
  {
    s_protocol->bind_transport(comm_send_wrapper);
  }

  s_active_protocol = cfg->protocol;
  return true;
}

void comm_deinit(void)
{
  if (s_protocol != NULL && s_protocol->deinit != NULL)
  {
    s_protocol->deinit();
  }

  if (s_transport != NULL && s_transport->deinit != NULL)
  {
    s_transport->deinit(NULL);
  }

  s_protocol = NULL;
  s_transport = NULL;
  s_active_protocol = COMM_PROTOCOL_NONE;
}

void comm_poll(void)
{
  if (s_transport != NULL && s_transport->poll_rx != NULL)
  {
    (void)s_transport->poll_rx(NULL, comm_rx_dispatch, NULL);
  }

  if (s_protocol != NULL && s_protocol->poll != NULL)
  {
    s_protocol->poll();
  }
}

void comm_can1_rx_isr(const can_rxbuf_type *rx)
{
  can_port_at32_rx_push_isr(rx);
}

comm_protocol_type_t comm_get_active_protocol(void)
{
  return s_active_protocol;
}
