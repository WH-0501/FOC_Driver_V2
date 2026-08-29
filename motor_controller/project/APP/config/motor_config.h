#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"
#include "motor_axis.h"

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

/** 多轴静态配置：axis[i] 对应 motor_axis_get(i) */
typedef struct
{
  motor_config_t axis[MOTOR_AXIS_COUNT];
} motor_system_config_t;

bool motor_config_register_storage(const motor_config_storage_ops_t *ops);
void motor_config_clear_storage(void);

/** 单轴默认参数（供 motor_config_set_axis_defaults 内部或单轴工具使用） */
void motor_config_set_defaults(motor_config_t *config);

/** 按轴号写入差异化默认值（CAN ID 等） */
void motor_config_set_axis_defaults(uint8_t axis_id, motor_config_t *config);

/** 全部轴回落默认 */
void motor_config_system_set_defaults(motor_system_config_t *config);

/**
 * 系统配置初始化：优先从存储加载 motor_system_config_t；
 * 未命中则各轴默认并尝试写回。
 */
bool motor_config_init(motor_system_config_t *config);

bool motor_config_load(motor_system_config_t *config);
bool motor_config_save(const motor_system_config_t *config);
bool motor_config_reset(motor_system_config_t *config);

#endif /* __MOTOR_CONFIG_H__ */
