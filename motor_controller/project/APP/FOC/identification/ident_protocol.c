#if defined(__CC_ARM)
#pragma Ospace
#endif
#include "ident_protocol.h"
#include <string.h>
static uint16_t read16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]*256u+p[1]); }
static void write32(uint8_t *p, uint32_t v) {
  p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v;
}
static void write_float(uint8_t *p, float v) { uint32_t b; memcpy(&b,&v,4); write32(p,b); }
bool ident_protocol_handle(motor_handle_t *m, uint8_t cmd, const uint8_t *p, uint8_t len,
                           void (*reply)(uint8_t, const uint8_t *, uint8_t)) {
  uint8_t out[28] = {1, 0};
  foc_ident_status_t s;
  error_t err = ERR_NONE;
  if (cmd < IDENT_CMD_CONTROL || cmd > IDENT_CMD_APPLY) return false;
  if (!reply) return true;
  if (!p || len < 1 || p[0] != 1) err = ERR_PARAM;
  else if (cmd == IDENT_CMD_CONTROL) {
    if (len < 2) err = ERR_PARAM;
    else if (p[1] == 0) foc_ident_stop(m);
    else if (p[1] > IDENT_ALL || len < 16) err = ERR_PARAM;
    else {
      ident_config_t c;
      c.current_a = read16(p+2)*0.001f; c.voltage_v = read16(p+4)*0.001f;
      c.injection_v = read16(p+6)*0.001f; c.injection_hz = (float)read16(p+8);
      c.speed_rad_s = read16(p+10)*0.01f; c.current_trip_a = read16(p+12)*0.001f;
      c.speed_trip_rad_s = read16(p+14)*0.01f;
      err = foc_ident_start(m, (ident_mode_t)p[1], &c);
    }
  } else if (cmd == IDENT_CMD_APPLY) err = foc_ident_apply(m);
  foc_ident_status(&s);
  out[1]=(uint8_t)err; out[2]=(uint8_t)s.stage; out[3]=(uint8_t)s.error; out[4]=s.result.valid;
  write32(out+8,s.elapsed_ms); write_float(out+12,s.result.rs);
  write_float(out+16,s.result.ld); write_float(out+20,s.result.lq); write_float(out+24,s.result.flux);
  reply(cmd,out,sizeof(out)); return true;
}
