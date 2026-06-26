#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

#ifndef MOTOR_CONFIG_MAX_STORAGE_BACKENDS
#define MOTOR_CONFIG_MAX_STORAGE_BACKENDS 4u // 最多支持 4 个存储后端(EEPROM / FLASH 等)
#endif

typedef struct
{
  /**
   * @brief 从后端读取指定长度字节流
   * @retval true 读取成功（内容合法性由上层校验）
   */
  bool (*read)(void *ctx, uint8_t *buf, uint32_t len);

  /**
   * @brief 向后端写入指定长度字节流
   * @retval true 写入成功
   */
  bool (*write)(void *ctx, const uint8_t *buf, uint32_t len);

  /**
   * @brief 擦除后端配置（可选）
   * @retval true 擦除成功
   */
  bool (*erase)(void *ctx);

  void *ctx;
} motor_config_storage_ops_t;

/**
 * @brief 注册一个配置存储后端（如 EEPROM / FLASH）
 * @retval true 注册成功
 */
bool motor_config_register_storage(const motor_config_storage_ops_t *ops);

/**
 * @brief 清空所有已注册存储后端
 */
void motor_config_clear_storage(void);

/**
 * @brief 生成一份默认电机配置
 */
void motor_config_set_defaults(motor_config_t *config);

/**
 * @brief 参数初始化入口：优先读取存储参数，若无则回落默认参数
 * @param[out] config 输出配置
 * @retval true 从存储恢复成功
 * @retval false 未命中存储，已回落默认参数
 */
bool motor_config_init(motor_config_t *config);

/**
 * @brief 读取电机配置
 * @param[out] config 输出配置
 * @retval true 从存储恢复成功
 * @retval false 未命中存储，返回默认配置
 */
bool motor_config_load(motor_config_t *config);

/**
 * @brief 保存电机配置
 * @retval true 保存成功
 */
bool motor_config_save(const motor_config_t *config);

/**
 * @brief 重置为默认配置并写回存储
 * @param[out] config 输出默认配置（可为 NULL）
 * @retval true 重置成功
 */
bool motor_config_reset(motor_config_t *config);

#endif /* __MOTOR_CONFIG_H__ */
