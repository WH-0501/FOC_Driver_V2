#include "motor_config_storage_ops.h"
#include "motor_config.h"
#include <string.h>

typedef struct
{
  uint32_t primary_addr;
  uint32_t backup_addr;
  uint32_t region_size;
} motor_cfg_region_t;

static const motor_cfg_region_t s_eeprom_region = {
  MOTOR_CFG_EEPROM_PRIMARY_ADDR,
  MOTOR_CFG_EEPROM_BACKUP_ADDR,
  MOTOR_CFG_EEPROM_REGION_SIZE
};

static const motor_cfg_region_t s_flash_region = {
  MOTOR_CFG_FLASH_PRIMARY_ADDR,
  MOTOR_CFG_FLASH_BACKUP_ADDR,
  MOTOR_CFG_FLASH_REGION_SIZE
};

static const motor_cfg_region_t s_flash_factory_region = {
  MOTOR_CFG_FLASH_FACTORY_ADDR,
  MOTOR_CFG_FLASH_FACTORY_ADDR, /* 出厂参数默认单副本 */
  MOTOR_CFG_FLASH_FACTORY_REGION_SIZE
};

static const motor_cfg_storage_adapter_t *s_adapter = NULL;

static bool motor_cfg_read_with_mirror(motor_cfg_raw_read_fn fn,
                                       const motor_cfg_region_t *region,
                                       uint8_t *buf, uint32_t len)
{
  if ((fn == NULL) || (region == NULL) || (buf == NULL) || (len == 0u))
  {
    return false;
  }
  if (len > region->region_size)
  {
    return false;
  }

  /* 先读主区，失败再回退到备份区 */
  if (fn(region->primary_addr, buf, len))
  {
    return true;
  }
  return fn(region->backup_addr, buf, len);
}

static bool motor_cfg_write_with_mirror(motor_cfg_raw_write_fn fn,
                                        const motor_cfg_region_t *region,
                                        const uint8_t *buf, uint32_t len)
{
  bool ok_primary;
  bool ok_backup;

  if ((fn == NULL) || (region == NULL) || (buf == NULL) || (len == 0u))
  {
    return false;
  }
  if (len > region->region_size)
  {
    return false;
  }

  /* 先写主区，再写备份区；任一写成功即认为本后端可用。 */
  ok_primary = fn(region->primary_addr, buf, len);
  ok_backup = fn(region->backup_addr, buf, len);
  return (ok_primary || ok_backup);
}

static bool motor_cfg_erase_with_mirror(motor_cfg_raw_erase_fn fn,
                                        const motor_cfg_region_t *region)
{
  bool ok_primary;
  bool ok_backup;

  if ((fn == NULL) || (region == NULL))
  {
    return false;
  }

  ok_primary = fn(region->primary_addr, region->region_size);
  ok_backup = fn(region->backup_addr, region->region_size);
  return (ok_primary || ok_backup);
}

static bool eeprom_storage_read(void *ctx, uint8_t *buf, uint32_t len)
{
  const motor_cfg_region_t *region = (const motor_cfg_region_t *)ctx;
  return motor_cfg_read_with_mirror((s_adapter != NULL) ? s_adapter->eeprom_read : NULL, region, buf, len);
}

static bool eeprom_storage_write(void *ctx, const uint8_t *buf, uint32_t len)
{
  const motor_cfg_region_t *region = (const motor_cfg_region_t *)ctx;
  return motor_cfg_write_with_mirror((s_adapter != NULL) ? s_adapter->eeprom_write : NULL, region, buf, len);
}

static bool eeprom_storage_erase(void *ctx)
{
  (void)ctx;
  return false; /* EEPROM 擦除通常由写流程隐式完成，模板默认不单独擦除。 */
}

static bool flash_storage_read(void *ctx, uint8_t *buf, uint32_t len)
{
  const motor_cfg_region_t *region = (const motor_cfg_region_t *)ctx;
  return motor_cfg_read_with_mirror((s_adapter != NULL) ? s_adapter->flash_read : NULL, region, buf, len);
}

static bool flash_storage_write(void *ctx, const uint8_t *buf, uint32_t len)
{
  const motor_cfg_region_t *region = (const motor_cfg_region_t *)ctx;
  bool erased;

  if (region == NULL)
  {
    return false;
  }

  /*
   * FLASH 先擦再写：
   * - 先尝试擦双区，任一成功即可继续；
   * - 写阶段仍采用主/备双写，任一成功即返回成功。
   */
  erased = motor_cfg_erase_with_mirror((s_adapter != NULL) ? s_adapter->flash_erase : NULL, region);
  if (!erased)
  {
    return false;
  }

  return motor_cfg_write_with_mirror((s_adapter != NULL) ? s_adapter->flash_write : NULL, region, buf, len);
}

static bool flash_storage_erase(void *ctx)
{
  const motor_cfg_region_t *region = (const motor_cfg_region_t *)ctx;
  return motor_cfg_erase_with_mirror((s_adapter != NULL) ? s_adapter->flash_erase : NULL, region);
}

static bool flash_factory_storage_read(void *ctx, uint8_t *buf, uint32_t len)
{
  const motor_cfg_region_t *region = (const motor_cfg_region_t *)ctx;

  if ((region == NULL) || (buf == NULL) || (len == 0u))
  {
    return false;
  }
  if (len > region->region_size)
  {
    return false;
  }

  if ((s_adapter == NULL) || (s_adapter->flash_read == NULL))
  {
    return false;
  }

  return s_adapter->flash_read(region->primary_addr, buf, len);
}

static const motor_config_storage_ops_t s_eeprom_ops = {
  .read = eeprom_storage_read,
  .write = eeprom_storage_write,
  .erase = eeprom_storage_erase,
  .ctx = (void *)&s_eeprom_region
};

static const motor_config_storage_ops_t s_flash_ops = {
  .read = flash_storage_read,
  .write = flash_storage_write,
  .erase = flash_storage_erase,
  .ctx = (void *)&s_flash_region
};

static const motor_config_storage_ops_t s_flash_factory_readonly_ops = {
  .read = flash_factory_storage_read,
  .write = NULL,
  .erase = NULL,
  .ctx = (void *)&s_flash_factory_region
};

bool motor_config_register_storage_ops_by_mode(motor_cfg_storage_mode_t mode,
                                               const motor_cfg_storage_adapter_t *adapter)
{
  bool ok = false;

  if (adapter == NULL)
  {
    return false;
  }

  /* 模式切换时清理旧注册，避免重复叠加。 */
  motor_config_clear_storage();
  s_adapter = adapter;

  switch (mode)
  {
    case MOTOR_CFG_STORAGE_MODE_EEPROM_ONLY:
      if ((s_adapter->eeprom_read == NULL) || (s_adapter->eeprom_write == NULL))
      {
        return false;
      }
      /* 仅 EEPROM：读写都走 EEPROM 主/备双副本。 */
      if (motor_config_register_storage(&s_eeprom_ops))
      {
        ok = true;
      }
      break;

    case MOTOR_CFG_STORAGE_MODE_EEPROM_WITH_FACTORY_FLASH:
      if ((s_adapter->eeprom_read == NULL) || (s_adapter->eeprom_write == NULL) ||
          (s_adapter->flash_read == NULL))
      {
        return false;
      }
      /*
       * EEPROM + 出厂 FLASH：
       * - 读：EEPROM 优先，失败回退到 FLASH 出厂参数
       * - 写：仅 EEPROM（FLASH 出厂参数后端为只读）
       */
      if (motor_config_register_storage(&s_eeprom_ops))
      {
        ok = true;
      }
      if (motor_config_register_storage(&s_flash_factory_readonly_ops))
      {
        ok = true;
      }
      break;

    case MOTOR_CFG_STORAGE_MODE_FLASH_ONLY:
      if ((s_adapter->flash_read == NULL) || (s_adapter->flash_erase == NULL) ||
          (s_adapter->flash_write == NULL))
      {
        return false;
      }
      /*
       * 仅 FLASH：
       * - 运行参数：FLASH 主/备双副本（可写）
       * - 出厂参数：FLASH 只读区（回退）
       */
      if (motor_config_register_storage(&s_flash_ops))
      {
        ok = true;
      }
      if (motor_config_register_storage(&s_flash_factory_readonly_ops))
      {
        ok = true;
      }
      break;

    default:
      break;
  }

  return ok;
}

bool motor_config_register_default_storage_ops(const motor_cfg_storage_adapter_t *adapter)
{
  return motor_config_register_storage_ops_by_mode(MOTOR_CFG_STORAGE_MODE_DEFAULT, adapter);
}
