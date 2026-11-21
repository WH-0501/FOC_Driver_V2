#include "sweep_protocol.h"
#if FOC_SWEEP_ENABLED
#include "common.h"
#include <string.h>
/* Wire v1 retained for the existing MotorTune device-sweep UI. Big-endian. */
static uint16_t read16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }
static void write16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void write32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static void write_float(uint8_t *p, float f)
{
  uint32_t bits;
  memcpy(&bits, &f, sizeof(bits)); write32(p, bits);
}
bool sweep_protocol_handle(motor_handle_t *m, uint8_t cmd, const uint8_t *p,
                           uint8_t length, void (*reply)(uint8_t, const uint8_t *, uint8_t))
{
  uint8_t out[56] = {1u, 0u};
  foc_sweep_rejection_t rejection = {0u, 0.0f};
  foc_sweep_status_t s;
  if (cmd < SWEEP_CMD_CONTROL || cmd > SWEEP_CMD_RESULT) return false;
  if (!reply) return true;
  if (cmd == SWEEP_CMD_CONTROL) {
    if (!p || length < 11u || p[0] != 1u || p[1] > SWEEP_CHIRP) out[1] = 2u;
    else if (!p[1]) foc_sweep_stop(m, SWEEP_STOP); /* stop never depends on a stale run id */
    else {
      sweep_config_t cfg;
      cfg.method = p[1]; cfg.count = p[8];
      cfg.low_hz = (float)read16(p + 4); cfg.high_hz = (float)read16(p + 6);
      cfg.peak = (float)read16(p + 9) * 0.001f;
      cfg.sample_hz = (float)FOC_CURRENT_LOOP_FS_HZ;
      out[1] = (uint8_t)foc_sweep_start_checked(m, &cfg, read16(p + 2), &rejection);
    }
  }
  foc_sweep_status(&s);
  if (cmd == SWEEP_CMD_RESULT) {
    sweep_result_t r;
    uint16_t requested_run = p && length >= 3u ? read16(p) : 0u;
    uint8_t index = p && length >= 3u ? p[2] : 0u;
    out[1] = p && length >= 3u ? (uint8_t)foc_sweep_result(requested_run, index, &r) : 2u;
    write16(out + 2, requested_run); out[4] = index; out[5] = s.state;
    write16(out + 6, s.flags);
    if (!out[1]) {
      write_float(out + 8, r.hz); write_float(out + 12, r.input);
      write_float(out + 16, r.output); write_float(out + 20, r.real);
      write_float(out + 24, r.imag); write_float(out + 28, r.coherence);
      write16(out + 32, r.blocks);
    }
    reply(cmd, out, 34u);
  } else {
    out[2] = s.state; out[3] = s.method; write16(out + 4, s.run);
    out[6] = s.count; out[7] = s.progress; write32(out + 8, s.sample_hz);
    write32(out + 12, s.samples); write16(out + 16, s.flags); write16(out + 18, s.max_us);
    write16(out + 20, s.interval_min_us); write16(out + 22, s.interval_max_us);
    write16(out + 24, s.interval_last_us); out[26] = s.timing_reason; out[27] = 0xa5;
    write32(out + 28, s.failure_sample);
    write16(out + 32, s.sense_max_us); write16(out + 34, s.control_max_us);
    write16(out + 36, s.engine_max_us); out[38] = 0xa6; out[39] = 1;
    if (out[1] && rejection.reason) {
      out[40] = rejection.reason; out[41] = 0xa7; out[42] = 1;
      write_float(out + 44, rejection.value);
      reply(cmd, out, 48u); /* only failed starts; successful/status frames unchanged */
    } else if (!out[1] && s.motion_reason) {
      /* A8/1 extends the unchanged 40-byte A6/1 status. Only motion aborts
       * carry it; old hosts ignore the tail. 56+2 bytes fit CAN-FD DLC 64. */
      out[40] = s.motion_reason; out[41] = 0xa8; out[42] = 1;
      out[43] = s.motion_index;
      write_float(out + 44, s.motion_value);
      write_float(out + 48, s.motion_limit);
      write_float(out + 52, s.motion_hz);
      reply(cmd, out, 56u);
    } else reply(cmd, out, 40u);
  }
  return true;
}
#else
/* A production build has no sweep RAM or ISR work. Explicitly answer unsupported
 * instead of leaving the host waiting for a typed reply after a dispatch ACK. */
bool sweep_protocol_handle(motor_handle_t *m, uint8_t cmd, const uint8_t *p,
                           uint8_t length, void (*reply)(uint8_t, const uint8_t *, uint8_t))
{
  uint8_t out[34] = {1u, 3u}; /* error 3: feature not compiled in */
  (void)m;
  if (cmd < SWEEP_CMD_CONTROL || cmd > SWEEP_CMD_RESULT) return false;
  if (cmd == SWEEP_CMD_CONTROL && p && length >= 4u) { out[4] = p[2]; out[5] = p[3]; }
  if (cmd == SWEEP_CMD_RESULT && p && length >= 3u) { out[2] = p[0]; out[3] = p[1]; out[4] = p[2]; }
  if (reply) reply(cmd, out, cmd == SWEEP_CMD_RESULT ? 34u : 20u);
  return true;
}
#endif
