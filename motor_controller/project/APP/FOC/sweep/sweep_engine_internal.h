#ifndef FOC_SWEEP_ENGINE_INTERNAL_H
#define FOC_SWEEP_ENGINE_INTERNAL_H
#include "sweep_engine.h"
/* Shared phase seed table stays in the engine: no duplicate table or RAM. */
void sweep_engine_reset_phase(sweep_engine_t *e, unsigned index);
#endif
