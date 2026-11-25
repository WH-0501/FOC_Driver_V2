#if defined(__CC_ARM)
#pragma Ospace
#endif
#include "foc_sweep.h"
#if FOC_SWEEP_ENABLED
#include "sweep_record.h"
#include "board.h"
#include <string.h>
#define CAPACITY 4u
#define PAIRS 6u
/* 224 bytes of wire-ready slots + 16 bytes of state/diagnostics. Full 20 kHz binary32 pairs.
 * Head/tail are monotonic uint8 counters; occupancy never exceeds CAPACITY. */
static union { uint32_t words[14]; uint8_t bytes[56]; } slots[CAPACITY];
static uint32_t wire32(uint32_t v)
{ return (v>>24) | ((v>>8)&0xff00u) | ((v<<8)&0xff0000u) | (v<<24); }
static volatile uint8_t head, tail;
static uint8_t used;
static bool enabled;
static uint16_t run_id;
static volatile uint32_t dropped;
static uint32_t submitted;
uint32_t sweep_record_dropped(void) { return dropped; }
uint32_t sweep_record_submitted(void) { return submitted; }
void sweep_record_begin(uint16_t run, bool on)
{
  /* Called with interrupts masked before publishing the sweep owner. */
  enabled = on; run_id = run; head = tail = used = 0;
  dropped = submitted = 0;
}
bool sweep_record_enabled(void) { return enabled; }
void sweep_record_end(void)
{
  if (!used) return;
  slots[head & (CAPACITY-1u)].bytes[1] = used;
  __DMB(); ++head; used = 0;
}
void sweep_record_push(uint32_t index, float input, float output)
{
  unsigned slot; uint32_t bits;
  if (!enabled) return;
  if ((uint8_t)(head-tail) >= CAPACITY) { ++dropped; return; }
  slot = head & (CAPACITY-1u);
  if (!used) {
    slots[slot].words[0] = wire32(0x01000000u | run_id);
    slots[slot].words[1] = wire32(index);
  }
  /* Two REV/store operations on Cortex-M4, no arithmetic/quantization.
   * A completed slot is already the protocol-v1 payload; retries do no packing. */
  memcpy(&bits, &input, 4); slots[slot].words[2u+2u*used] = wire32(bits);
  memcpy(&bits, &output, 4); slots[slot].words[3u+2u*used] = wire32(bits);
  if (++used == PAIRS) sweep_record_end();
}
bool sweep_record_poll(uint32_t node,
    bool (*send)(uint32_t, uint8_t, const uint8_t *, uint8_t))
{
  unsigned slot;
  if (head == tail) return false;
  __DMB(); slot = tail & (CAPACITY-1u);
  /* Retry the same immutable wire image if the shared transport is busy. */
  if (send && send(node, 0xBFu, slots[slot].bytes,
                  (uint8_t)(8u+8u*slots[slot].bytes[1]))) {
    submitted += slots[slot].bytes[1]; __DMB(); ++tail; return true;
  }
  return false;
}
#endif
