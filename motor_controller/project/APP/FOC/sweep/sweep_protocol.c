/* Foreground-only module: reduce code size without changing floating-point mode. */
#if defined(__CC_ARM)
#pragma Ospace
#endif
#include "sweep_protocol.h"
#if FOC_SWEEP_ENABLED
#include "common.h"
#include "sweep_record.h"
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
  uint8_t out[62] = {1u, 0u};
  foc_sweep_rejection_t rejection = {0u, 0.0f};
  foc_sweep_status_t s;
  if (cmd < SWEEP_CMD_CONTROL || cmd > SWEEP_CMD_RESULT) return false;
  if (!reply) return true;
  if (cmd == SWEEP_CMD_CONTROL) {
    if (!p || length < 11u || p[0] != 1u || p[1] > SWEEP_CHIRP) out[1] = 2u;
    else if (!p[1]) foc_sweep_stop(m, SWEEP_STOP); /* stop never depends on a stale run id */
    else {
      sweep_config_t cfg;
      unsigned i;
      memset(&cfg,0,sizeof(cfg));
      cfg.method = p[1];
      cfg.count = p[8];
      cfg.low_hz = (float)read16(p + 4); 
      cfg.high_hz = (float)read16(p + 6);
      cfg.peak = (float)read16(p + 9) * 0.001f;
      cfg.sample_hz = (float)FOC_CURRENT_LOOP_FS_HZ; ///< 扫频采样率. 固定为电流环采样率
      /* Echo the queried layout before enabling: an old host must not start
       * a long capture and interpret it as its fixed four-second window. */
      if(cfg.method==SWEEP_CHIRP && (length<19u || p[13]!=0xaeu || p[14]!=1u ||
         read16(p+15)!=SWEEP_CHIRP_PERIOD_MS || p[17]!=SWEEP_CHIRP_SETTLE_PERIODS ||
         p[18]!=SWEEP_CHIRP_MEASURE_PERIODS)) out[1]=2u;
      if(cfg.method==SWEEP_STEPPED) {
        if(cfg.count>SWEEP_MAX_POINTS || length<17u+2u*cfg.count ||
           p[13]!=0xadu || p[14]!=1u || read16(p+15)!=FOC_CURRENT_LOOP_FS_HZ) out[1]=2u;
        else for(i=0;i<cfg.count;++i) cfg.periods[i]=read16(p+17u+2u*i);
      }
      if(!out[1]) out[1] = (uint8_t)foc_sweep_start_recorded(m, &cfg, read16(p + 2), &rejection,
                                length >= 13u && p[11] == 0xa9u && p[12] == 1u);
    }
  }
  foc_sweep_status(&s);
  if (cmd == SWEEP_CMD_RESULT) {
    /* Retired device estimator: typed unsupported response, never fake H1.
     * Old hosts can still stop/query and decode recording frames. */
    uint16_t requested_run = p && length >= 3u ? read16(p) : 0u;
    out[1] = p && length >= 3u ? 3u : 2u;
    write16(out+2, requested_run); out[4]=p && length>=3u?p[2]:0u;
    out[5]=s.state; write16(out+6,s.flags);
    reply(cmd,out,34u);
  } else {
    /* AE/1 also accompanies errors and motion aborts; bytes 0..55 retain
     * their previous meaning. 62 payload bytes + 2 transport bytes = DLC 64. */
    out[56]=0xaeu; out[57]=1u;
    write16(out+58,SWEEP_CHIRP_PERIOD_MS);
    out[60]=SWEEP_CHIRP_SETTLE_PERIODS; out[61]=SWEEP_CHIRP_MEASURE_PERIODS;
    out[2] = s.state; out[3] = s.method; write16(out + 4, s.run);
    out[6] = s.count; out[7] = s.progress; write32(out + 8, s.sample_hz?s.sample_hz:FOC_CURRENT_LOOP_FS_HZ);
    write32(out + 12, s.samples); write16(out + 16, s.flags); write16(out + 18, s.max_us);
    write16(out + 20, s.interval_min_us); write16(out + 22, s.interval_max_us);
    write16(out + 24, s.interval_last_us); out[26] = s.timing_reason; out[27] = 0xa5;
    write32(out + 28, s.failure_sample);
    write16(out + 32, s.sense_max_us); write16(out + 34, s.control_max_us);
    write16(out + 36, s.engine_max_us); out[38] = 0xa6; out[39] = 1;
    if (out[1] && rejection.reason) {
      out[40] = rejection.reason; out[41] = 0xa7; out[42] = 1;
      write_float(out + 44, rejection.value);
      reply(cmd, out, 62u);
    } else if (!out[1] && s.motion_reason) {
      /* A8/1 extends the unchanged 40-byte A6/1 status. Only motion aborts
       * carry it; old hosts ignore the tail. 56+2 bytes fit CAN-FD DLC 64. */
      out[40] = s.motion_reason; out[41] = 0xa8; out[42] = 1;
      out[43] = s.motion_index;
      write_float(out + 44, s.motion_value);
      write_float(out + 48, s.motion_limit);
      write_float(out + 52, s.motion_hz);
      reply(cmd, out, 62u);
    } else {
      out[40]=0xa9u; out[41]=1u; out[42]=(uint8_t)sweep_record_enabled();
      /* AA marker distinguishes diagnostics from old zero DLC padding. Counts
       * are a status snapshot; submitted means CAN-owned, not host-received. */
      out[43]=0xaau;
      write32(out+44, sweep_record_dropped());
      write32(out+48, sweep_record_submitted());
      /* Optional capability tail. Bytes 0..51 and raw 0xBF are unchanged.
       * AC/1, bit0: host H1; bit1: AD/1 stepped plan; bit2: AE/1 timing required. Byte55: total periods / stepped blocks. */
      out[52]=0xacu; out[53]=1u; out[54]=7u;
      out[55]=(uint8_t)(s.method==SWEEP_STEPPED ? SWEEP_STEPPED_BLOCKS : SWEEP_CHIRP_TOTAL_PERIODS);
      reply(cmd, out, 62u);
    }
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
