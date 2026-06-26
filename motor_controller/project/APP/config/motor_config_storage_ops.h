#ifndef __MOTOR_CONFIG_STORAGE_OPS_H__
#define __MOTOR_CONFIG_STORAGE_OPS_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * 存储地址规划（模板）：
 * 1) EEPROM：主区 + 备份区（双副本）
 * 2) FLASH ：主区 + 备份区（双副本，按实际扇区边界对齐）
 *
 * 说明：
 * - 以下地址仅为模板示例，请按实际芯片容量与分区表调整。
 * - 每个区容量需 >= motor_config 模块传入的 blob 长度。
 */
#ifndef MOTOR_CFG_EEPROM_PRIMARY_ADDR
#define MOTOR_CFG_EEPROM_PRIMARY_ADDR (0x0000u)
#endif
#ifndef MOTOR_CFG_EEPROM_BACKUP_ADDR
#define MOTOR_CFG_EEPROM_BACKUP_ADDR  (0x0200u)
#endif
#ifndef MOTOR_CFG_EEPROM_REGION_SIZE
#define MOTOR_CFG_EEPROM_REGION_SIZE  (0x0200u) /* 512 bytes */
#endif

#ifndef MOTOR_CFG_FLASH_PRIMARY_ADDR
#define MOTOR_CFG_FLASH_PRIMARY_ADDR  (0x0800F000u)
#endif
#ifndef MOTOR_CFG_FLASH_BACKUP_ADDR
#define MOTOR_CFG_FLASH_BACKUP_ADDR   (0x0800F800u)
#endif
#ifndef MOTOR_CFG_FLASH_REGION_SIZE
#define MOTOR_CFG_FLASH_REGION_SIZE   (0x0800u) /* 2KB，按实际扇区调整 */
#endif

#ifndef MOTOR_CFG_FLASH_FACTORY_ADDR
#define MOTOR_CFG_FLASH_FACTORY_ADDR  (0x0800E800u)
#endif
#ifndef MOTOR_CFG_FLASH_FACTORY_REGION_SIZE
#define MOTOR_CFG_FLASH_FACTORY_REGION_SIZE (0x0800u) /* 2KB，按实际扇区调整 */
#endif

typedef enum
{
  MOTOR_CFG_STORAGE_MODE_EEPROM_ONLY = 0,              /* 仅 EEPROM（运行参数） */
  MOTOR_CFG_STORAGE_MODE_EEPROM_WITH_FACTORY_FLASH,    /* EEPROM 运行参数 + FLASH 出厂参数只读回退 */
  MOTOR_CFG_STORAGE_MODE_FLASH_ONLY,                   /* FLASH 运行参数 + 备份 + 出厂参数只读回退 */
} motor_cfg_storage_mode_t;

typedef bool (*motor_cfg_raw_read_fn)(uint32_t addr, uint8_t *buf, uint32_t len);
typedef bool (*motor_cfg_raw_write_fn)(uint32_t addr, const uint8_t *buf, uint32_t len);
typedef bool (*motor_cfg_raw_erase_fn)(uint32_t addr, uint32_t len);

typedef struct
{
  /* EEPROM 后端（可选，按模式决定是否必需） */
  motor_cfg_raw_read_fn eeprom_read;
  motor_cfg_raw_write_fn eeprom_write;

  /* FLASH 后端（可选，按模式决定是否必需） */
  motor_cfg_raw_read_fn flash_read;
  motor_cfg_raw_erase_fn flash_erase;
  motor_cfg_raw_write_fn flash_write;
} motor_cfg_storage_adapter_t;

#ifndef MOTOR_CFG_STORAGE_MODE_DEFAULT
#define MOTOR_CFG_STORAGE_MODE_DEFAULT MOTOR_CFG_STORAGE_MODE_FLASH_ONLY
#endif

/**
 * @brief 按指定模式注册模板存储后端
 * @param mode 存储模式
 * @param adapter 显式注入底层适配器（不可为 NULL）
 * @retval true 至少注册成功一个后端
 */
bool motor_config_register_storage_ops_by_mode(motor_cfg_storage_mode_t mode,
                                               const motor_cfg_storage_adapter_t *adapter);

/**
 * @brief 按默认模式注册模板存储后端
 * @param adapter 显式注入底层适配器（不可为 NULL）
 * @retval true 至少注册成功一个后端
 */
bool motor_config_register_default_storage_ops(const motor_cfg_storage_adapter_t *adapter);

#endif /* __MOTOR_CONFIG_STORAGE_OPS_H__ */
