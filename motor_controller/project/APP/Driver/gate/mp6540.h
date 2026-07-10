#ifndef __MP6540_H__
#define __MP6540_H__

#include <stdint.h>
#include <stdbool.h>
#include "gate_driver.h"

#define MP6540_CAL_DELAY_MS      10U
#define MP6540_WAKEUP_DELAY_MS   1U

typedef struct
{
    gate_driver_hw_cfg_t hw;
    bool is_sleep;
} MP6540_Handle_t;

void MP6540_Init(MP6540_Handle_t *hmp6, const gate_driver_hw_cfg_t *hw_cfg);
const gate_driver_t *MP6540_AsGateDriver(MP6540_Handle_t *hmp6);
void MP6540_RegisterAsActiveDriver(MP6540_Handle_t *hmp6);

#endif /* __MP6540_H__ */
