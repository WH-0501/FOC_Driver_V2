#ifndef FOC_IDENTIFICATION_H
#define FOC_IDENTIFICATION_H
#include "ident_engine.h"
#include "datatypes.h"
typedef struct {
  ident_stage_t stage;
  ident_error_t error;
  ident_result_t result;
  uint32_t elapsed_ms;
} foc_ident_status_t;
void foc_ident_defaults(motor_handle_t *m, ident_config_t *cfg);
error_t foc_ident_start(motor_handle_t *m, ident_mode_t mode, const ident_config_t *cfg);
void foc_ident_stop(motor_handle_t *m);
void foc_ident_on_disarm(motor_handle_t *m);
bool foc_ident_active(const motor_handle_t *m);
void foc_ident_run_isr(motor_handle_t *m);
void foc_ident_status(foc_ident_status_t *status);
error_t foc_ident_apply(motor_handle_t *m);
#endif
