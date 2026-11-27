#ifndef IDENT_PROTOCOL_H
#define IDENT_PROTOCOL_H
#include "foc_identification.h"
#define IDENT_CMD_CONTROL 0xC0u
#define IDENT_CMD_STATUS  0xC1u
#define IDENT_CMD_APPLY   0xC2u
bool ident_protocol_handle(motor_handle_t *m, uint8_t cmd, const uint8_t *p, uint8_t len,
                           void (*reply)(uint8_t, const uint8_t *, uint8_t));
#endif
