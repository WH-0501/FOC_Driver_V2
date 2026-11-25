#ifndef SWEEP_RECORD_H
#define SWEEP_RECORD_H
#include <stdbool.h>
#include <stdint.h>
/* Single ISR producer, foreground consumer. No heap, decimation or quantization.
 * A full queue drops new samples; original indices expose every gap to the host.
 * Capture failure does NOT modify the independent on-device H1 estimator. */
void sweep_record_begin(uint16_t run, bool enabled);
void sweep_record_push(uint32_t index, float input, float output);
void sweep_record_end(void);
bool sweep_record_enabled(void);
uint32_t sweep_record_dropped(void);
uint32_t sweep_record_submitted(void);
/* True only when one frame was accepted, false on empty or backpressure. */
bool sweep_record_poll(uint32_t node,
    bool (*send)(uint32_t, uint8_t, const uint8_t *, uint8_t));
#endif
