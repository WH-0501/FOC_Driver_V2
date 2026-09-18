#include "can_fd_custom_iap.h"
#include "iap_upgrade.h"
#include "mc_interface.h"
#include <stddef.h>

static bool (*s_send_frame)(uint32_t id, uint8_t cmd, const uint8_t *payload, uint8_t payload_len);
static uint16_t *s_node_id;
static void (*s_cycle_set)(uint8_t enable);

static bool iap_send(uint8_t *data, uint16_t length)
{
  if ((s_send_frame == NULL) || (s_node_id == NULL) || (data == NULL) || (length < 1u))
  {
    return false;
  }

  return s_send_frame((uint32_t)(*s_node_id), data[0], &data[1], (uint8_t)(length - 1u));
}

static void iap_motor(uint8_t enable)
{
  if (enable == 0u)
  {
    (void)mc_disable();
  }
}

static void iap_cycle(uint8_t enable)
{
  if (s_cycle_set != NULL)
  {
    s_cycle_set(enable);
  }
}

void can_fd_iap_setup(bool (*send_frame)(uint32_t id, uint8_t cmd, const uint8_t *payload,
                                         uint8_t payload_len),
                      uint16_t *node_id, void (*cycle_set)(uint8_t enable))
{
  IAP_Comm_t *comm;

  s_send_frame = send_frame;
  s_node_id = node_id;
  s_cycle_set = cycle_set;

  comm = IAP_Init();
  comm->send = iap_send;
  comm->motor = iap_motor;
  comm->cycle = iap_cycle;
}

bool can_fd_iap_is_active(void)
{
  return IAP_IsActive();
}

void can_fd_iap_on_rx(const comm_frame_t *frame)
{
  if ((frame == NULL) || (frame->data == NULL) || (frame->len < 2u))
  {
    return;
  }

  IAP_Process(frame->data, frame->len);
}

void can_fd_iap_poll(void)
{
  IAP_Error();
}
