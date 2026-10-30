#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

#ifndef MOTOR_CONFIG_MAX_STORAGE_BACKENDS
#define MOTOR_CONFIG_MAX_STORAGE_BACKENDS 4u
#endif

typedef struct
{
  bool (*read)(void *ctx, uint8_t *buf, uint32_t len);
  bool (*write)(void *ctx, const uint8_t *buf, uint32_t len);
  bool (*erase)(void *ctx);
  void *ctx;
} motor_config_storage_ops_t;

bool motor_config_register_storage(const motor_config_storage_ops_t *ops);
void motor_config_clear_storage(void);

void motor_config_set_defaults(motor_config_t *config);
bool motor_config_init(motor_config_t *config);
bool motor_config_load(motor_config_t *config);
bool motor_config_save(const motor_config_t *config);
bool motor_config_reset(motor_config_t *config);
/** 仅将三环 PID 恢复为默认值并写入 Flash；保留 param/limits/motion/comm 等。 */
bool motor_config_reset_pid(motor_config_t *config);

#endif /* __MOTOR_CONFIG_H__ */
