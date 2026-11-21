#ifndef FOC_SWEEP_PROTOCOL_H
#define FOC_SWEEP_PROTOCOL_H
#include "foc_sweep.h"
#define SWEEP_CMD_CONTROL 0xBCu
#define SWEEP_CMD_STATUS  0xBDu
#define SWEEP_CMD_RESULT  0xBEu
/* Foreground only, validated unicast requests; callback retains existing TX queue. */
bool sweep_protocol_handle(motor_handle_t *m, uint8_t cmd, const uint8_t *payload,
                           uint8_t length, void (*reply)(uint8_t, const uint8_t *, uint8_t));
#endif
