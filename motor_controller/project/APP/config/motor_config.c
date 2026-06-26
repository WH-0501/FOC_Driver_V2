#include "motor_config.h"
#include "common.h"
#include <string.h>

#define MOTOR_CONFIG_BLOB_MAGIC   (0x4D434647u) /* 'MCFG' */
#define MOTOR_CONFIG_BLOB_VERSION (1u)

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t payload_len;
  uint32_t checksum;
} motor_config_blob_header_t;

typedef struct
{
  motor_config_blob_header_t header;
  uint8_t payload[sizeof(motor_config_t)];
} motor_config_blob_t;

static const motor_config_storage_ops_t *s_storage_ops[MOTOR_CONFIG_MAX_STORAGE_BACKENDS];
static unsigned int s_storage_count = 0u;

static uint32_t motor_config_checksum32(const uint8_t *data, uint32_t len)
{
  uint32_t hash = 2166136261u; /* FNV-1a seed */
  uint32_t i;

  for (i = 0u; i < len; ++i)
  {
    hash ^= (uint32_t)data[i];
    hash *= 16777619u;
  }

  return hash;
}

static void motor_config_blob_build(motor_config_blob_t *blob, const motor_config_t *config)
{
  if ((blob == NULL) || (config == NULL))
  {
    return;
  }

  memset(blob, 0, sizeof(*blob));
  blob->header.magic = MOTOR_CONFIG_BLOB_MAGIC;
  blob->header.version = MOTOR_CONFIG_BLOB_VERSION;
  blob->header.payload_len = (uint16_t)sizeof(motor_config_t);
  memcpy(blob->payload, config, sizeof(motor_config_t));
  blob->header.checksum = motor_config_checksum32(blob->payload, sizeof(motor_config_t));
}

static bool motor_config_blob_parse(const motor_config_blob_t *blob, motor_config_t *config)
{
  uint32_t checksum;

  if ((blob == NULL) || (config == NULL))
  {
    return false;
  }

  if (blob->header.magic != MOTOR_CONFIG_BLOB_MAGIC)
  {
    return false;
  }
  if (blob->header.version != MOTOR_CONFIG_BLOB_VERSION)
  {
    return false;
  }
  if (blob->header.payload_len != (uint16_t)sizeof(motor_config_t))
  {
    return false;
  }

  checksum = motor_config_checksum32(blob->payload, (uint32_t)blob->header.payload_len);
  if (checksum != blob->header.checksum)
  {
    return false;
  }

  memcpy(config, blob->payload, sizeof(motor_config_t));
  return true;
}

static bool motor_config_has_storage(void)
{
  return (s_storage_count > 0u);
}

bool motor_config_register_storage(const motor_config_storage_ops_t *ops)
{
  if ((ops == NULL) || (ops->read == NULL))
  {
    return false;
  }

  if (s_storage_count >= MOTOR_CONFIG_MAX_STORAGE_BACKENDS)
  {
    return false;
  }

  s_storage_ops[s_storage_count++] = ops;
  return true;
}

void motor_config_clear_storage(void)
{
  memset(s_storage_ops, 0, sizeof(s_storage_ops));
  s_storage_count = 0u;
}

void motor_config_set_defaults(motor_config_t *config)
{
  if (config == NULL)
  {
    return;
  }

  memset(config, 0, sizeof(*config));

  config->param.pole_pairs = POLE_PAIRS;
  config->param.direction = 1;
  config->param.gear_ratio = GEAR_RATIO;
  config->param.encoder_counts_per_rev = ENCODER_COUNTS_PER_REV;
  config->param.theta_elec_offset_rad = THETA_ELEC_OFFSET_RAD;
  config->param.theta_offset_rad = THETA_OFFSET_RAD;

  config->limits.vbus_nominal = BUS_VOLTAGE;
  config->limits.vbus_uv_threshold = DEF_UVLO_VOLTAGE;
  config->limits.vbus_ov_threshold = DEF_OVP_VOLTAGE;
  config->limits.id_limit = DEF_OCP_CURRENT;
  config->limits.iq_limit = DEF_OCP_CURRENT;
  config->limits.vd_limit = BUS_VOLTAGE * 0.5f;
  config->limits.vq_limit = BUS_VOLTAGE * 0.5f;

  config->motion.max_speed_rad_s = MAX_VELOCITY_RAD_S;
  config->motion.max_accel_rad_s2 = MAX_ACC_RAD_S2;
  config->motion.max_jerk_rad_s3 = MAX_JERK_RAD_S3;

  config->pid.id.kp = DEF_D_KP;
  config->pid.id.ki = DEF_D_KI;
  config->pid.id.kd = DEF_D_KD;
  config->pid.id.ramp = DEF_D_RAMP;
  config->pid.id.limit = config->limits.vd_limit;
  config->pid.id.d_filter = DEF_D_D_FILTER;

  config->pid.iq.kp = DEF_Q_KP;
  config->pid.iq.ki = DEF_Q_KI;
  config->pid.iq.kd = DEF_Q_KD;
  config->pid.iq.ramp = DEF_Q_RAMP;
  config->pid.iq.limit = config->limits.vq_limit;
  config->pid.iq.d_filter = DEF_Q_D_FILTER; 

  config->pid.velocity.kp = DEF_VEL_KP;
  config->pid.velocity.ki = DEF_VEL_KI;
  config->pid.velocity.kd = DEF_VEL_KD;
  config->pid.velocity.ramp = DEF_VEL_RAMP;
  config->pid.velocity.limit = config->limits.iq_limit;
  config->pid.velocity.d_filter = DEF_VEL_D_FILTER;

  config->pid.position.kp = DEF_POS_KP;
  config->pid.position.ki = DEF_POS_KI;
  config->pid.position.kd = DEF_POS_KD;
  config->pid.position.ramp = DEF_POS_RAMP;
  config->pid.position.limit = config->limits.iq_limit;
  config->pid.position.d_filter = DEF_POS_D_FILTER;

  config->comm.can_id = 0x01u;
  config->auto_align_electrical = false;
}

bool motor_config_init(motor_config_t *config)
{
  bool loaded_from_storage;

  loaded_from_storage = motor_config_load(config);
  if (!loaded_from_storage)
  {
    /* 首次上电无有效参数时，回落默认参数并回写一次。 */
    (void)motor_config_save(config);
  }

  return loaded_from_storage;
}

bool motor_config_load(motor_config_t *config)
{
  unsigned int i;
  motor_config_blob_t blob;

  if (config == NULL)
  {
    return false;
  }

  for (i = 0u; i < s_storage_count; ++i)
  {
    const motor_config_storage_ops_t *ops = s_storage_ops[i];

    if ((ops != NULL) && (ops->read != NULL) &&
        ops->read(ops->ctx, (uint8_t *)&blob, (uint32_t)sizeof(blob)) &&
        motor_config_blob_parse(&blob, config))
    {
      return true;
    }
  }

  motor_config_set_defaults(config);
  return false;
}

bool motor_config_save(const motor_config_t *config)
{
  unsigned int i;
  bool all_ok = true;
  bool any_write = false;
  motor_config_blob_t blob;

  if (config == NULL)
  {
    return false;
  }

  motor_config_blob_build(&blob, config);

  if (!motor_config_has_storage())
  {
    return false;
  }

  for (i = 0u; i < s_storage_count; ++i)
  {
    const motor_config_storage_ops_t *ops = s_storage_ops[i];

    if ((ops == NULL) || (ops->write == NULL))
    {
      continue;
    }

    any_write = true;
    if (!ops->write(ops->ctx, (const uint8_t *)&blob, (uint32_t)sizeof(blob)))
    {
      all_ok = false;
    }
  }

  return (any_write && all_ok);
}

bool motor_config_reset(motor_config_t *config)
{
  motor_config_t defaults;
  bool saved_ok;

  motor_config_set_defaults(&defaults);
  saved_ok = motor_config_save(&defaults);

  if (config != NULL)
  {
    *config = defaults;
  }

  /* 无存储后端时也认为 reset 成功（至少运行时拿到了默认值） */
  if (!motor_config_has_storage())
  {
    return true;
  }

  return saved_ok;
}
