#include "can_fd_custom.h"
#include "comm.h"
#include "can_fd_custom_codec.h"
#include "can_fd_custom_iap.h"
#include "mc_interface.h"
#include "motor.h"
#include "motor_config.h"
#include "foc_fault.h"
#include "board.h"
#include "common.h"

static uint16_t s_node_id;
static bool (*s_send)(const comm_frame_t *frame);
static uint32_t s_last_report_us;
static uint8_t s_tx_cnt;
static uint32_t s_report_interval_us = 1000u;
static float s_mcu_temp_cal_c;
static motor_handle_t *s_motor;
static uint8_t s_tx_buf[COMM_FRAME_MAX_LEN];

/**
 * @brief 发送 CAN-FD 帧
 * 
 * @param id CAN-FD ID
 * @param cmd 命令
 * @param payload 有效数据
 * @param payload_len 有效数据长度
 * @return true 成功
 * @return false 失败
 */
static bool can_fd_custom_send_frame(uint32_t id, uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  comm_frame_t frame;
  uint8_t total_len;
  uint8_t padded_len;
  uint8_t i;

  if (s_send == NULL)
  {
    return false;
  }

  total_len = (uint8_t)(2u + payload_len);
  padded_len = can_fd_codec_pad_canfd_len(total_len);
  if (padded_len > COMM_FRAME_MAX_LEN)
  {
    return false;
  }

  s_tx_buf[0] = s_tx_cnt++;
  s_tx_buf[1] = cmd;
  for (i = 0u; i < payload_len; ++i)
  {
    s_tx_buf[2u + i] = payload[i];
  }
  for (i = total_len; i < padded_len; ++i)
  {
    s_tx_buf[i] = 0u;
  }

  frame.id = id;
  frame.len = padded_len;
  frame.data = s_tx_buf;

  return s_send(&frame);
}

/**
 * @brief 发送应答帧
 * 
 * @param cmd 命令
 * @param payload 有效数据
 * @param payload_len 有效数据长度
 */
static void can_fd_custom_reply(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  (void)can_fd_custom_send_frame(s_node_id, cmd, payload, payload_len);
}

static bool can_fd_custom_require_disabled(void)
{
  return !mc_is_enabled();
}

static bool can_fd_custom_motor_id_match(uint8_t motor_id)
{
  if (motor_id == 0u)
  {
    return true;
  }

  return (motor_id == (uint8_t)(s_node_id & 0xFFu));
}

static control_mode_t can_fd_custom_wire_mode_to_ctrl(uint8_t wire_mode)
{
  if (wire_mode >= (uint8_t)CTRL_MODE_RESERVED)
  {
    return CTRL_MODE_IDLE;
  }
  return (control_mode_t)wire_mode;
}

static uint8_t can_fd_custom_ctrl_to_wire_mode(control_mode_t mode)
{
  if (mode >= CTRL_MODE_RESERVED)
  {
    return (uint8_t)CTRL_MODE_IDLE;
  }
  return (uint8_t)mode;
}

static bool can_fd_custom_is_for_this_node(uint32_t id)
{
  if (id == CAN_FD_BROADCAST_ID)
  {
    return true;
  }

  if (id == CAN_FD_IAP_ID)
  {
    return true;
  }

  return (id == (uint32_t)s_node_id);
}

static void can_fd_custom_apply_pos(motor_handle_t *m, int16_t pos_raw)
{
  m->ref.position_rad = can_fd_codec_pos_q7_to_rad(pos_raw);
}

static void can_fd_custom_apply_vel(motor_handle_t *m, int16_t vel_raw)
{
  m->ref.speed_rad_s = can_fd_codec_vel_q7_rpm_to_rad_s(vel_raw);
}

static void can_fd_custom_apply_curr(motor_handle_t *m, int16_t curr_ma)
{
  /// TODO: 根据上层传递的参数类型,当前为电流,直接赋值 iq,若为 扭矩值, 则赋值 torque_nm
  m->ref.iq = (float)curr_ma / 1000.0f;
}

static void can_fd_custom_parse_entries(const uint8_t *payload, uint8_t payload_len, uint8_t entry_size,
                                        void (*apply_entry)(motor_handle_t *m, const uint8_t *entry))
{
  motor_handle_t *m = motor_get_handler();
  uint8_t offset = 0u;

  if (apply_entry == NULL)
  {
    return;
  }

  while ((uint8_t)(offset + entry_size) <= payload_len)
  {
    const uint8_t *entry = &payload[offset];

    if (can_fd_custom_motor_id_match(entry[0]))
    {
      apply_entry(m, entry);
    }

    offset = (uint8_t)(offset + entry_size);
  }
}

static void can_fd_custom_apply_pos_entry(motor_handle_t *m, const uint8_t *entry)
{
  can_fd_custom_apply_pos(m, can_fd_codec_read_i16_be(&entry[1]));
}

static void can_fd_custom_apply_vel_entry(motor_handle_t *m, const uint8_t *entry)
{
  can_fd_custom_apply_vel(m, can_fd_codec_read_i16_be(&entry[1]));
}

static void can_fd_custom_apply_curr_entry(motor_handle_t *m, const uint8_t *entry)
{
  can_fd_custom_apply_curr(m, can_fd_codec_read_i16_be(&entry[1]));
}

static void can_fd_custom_apply_pos_vel_entry(motor_handle_t *m, const uint8_t *entry)
{
  can_fd_custom_apply_pos(m, can_fd_codec_read_i16_be(&entry[1]));
  can_fd_custom_apply_vel(m, can_fd_codec_read_i16_be(&entry[3]));
}

static void can_fd_custom_apply_pos_curr_entry(motor_handle_t *m, const uint8_t *entry)
{
  can_fd_custom_apply_pos(m, can_fd_codec_read_i16_be(&entry[1]));
  can_fd_custom_apply_curr(m, can_fd_codec_read_i16_be(&entry[3]));
}

static void can_fd_custom_apply_vel_curr_entry(motor_handle_t *m, const uint8_t *entry)
{
  can_fd_custom_apply_vel(m, can_fd_codec_read_i16_be(&entry[1]));
  can_fd_custom_apply_curr(m, can_fd_codec_read_i16_be(&entry[3]));
}

static void can_fd_custom_apply_pos_vel_curr_entry(motor_handle_t *m, const uint8_t *entry)
{
  can_fd_custom_apply_pos(m, can_fd_codec_read_i16_be(&entry[1]));
  can_fd_custom_apply_vel(m, can_fd_codec_read_i16_be(&entry[3]));
  can_fd_custom_apply_curr(m, can_fd_codec_read_i16_be(&entry[5]));
}

static void can_fd_custom_handle_pos_vel_curr(const uint8_t *payload, uint8_t payload_len)
{
  uint8_t offset = 0u;

  if (payload_len >= 9u)
  {
    uint8_t total_frames = payload[0];
    uint8_t frame_index = payload[1];

    if ((total_frames >= 1u) && (total_frames <= 8u) && (frame_index < total_frames) &&
        (((payload_len - 2u) % 7u) == 0u))
    {
      offset = 2u;
    }
  }

  can_fd_custom_parse_entries(&payload[offset], (uint8_t)(payload_len - offset), 7u,
                            can_fd_custom_apply_pos_vel_curr_entry);
}

static void can_fd_custom_reply_pid(uint8_t reply_cmd, const pid_param_t *pid)
{
  uint8_t payload[6];

  if (pid == NULL)
  {
    return;
  }

  can_fd_codec_pack_pid(pid->kp, pid->ki, pid->kd, payload);
  can_fd_custom_reply(reply_cmd, payload, 6u);
}

static void can_fd_custom_handle_command(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  motor_handle_t *m = s_motor;
  uint8_t reply_payload[6];
  uint16_t hz;
  int16_t temp_cal_raw;
  uint8_t node_id_be[2];
  int result;

  if (m == NULL)
  {
    return;
  }

  if (can_fd_iap_is_active() && (cmd < CAN_FD_CMD_IAP_ENTER))
  {
    return;
  }

  switch (cmd)
  {
    case CAN_FD_CMD_SYNC:
      s_last_report_us = board_time_us();
      break;

    case CAN_FD_CMD_TAR_POS:
      can_fd_custom_parse_entries(payload, payload_len, 3u, can_fd_custom_apply_pos_entry);
      break;

    case CAN_FD_CMD_TAR_VEL:
      can_fd_custom_parse_entries(payload, payload_len, 3u, can_fd_custom_apply_vel_entry);
      break;

    case CAN_FD_CMD_TAR_CURR:
      can_fd_custom_parse_entries(payload, payload_len, 3u, can_fd_custom_apply_curr_entry);
      break;

    case CAN_FD_CMD_TAR_POS_VEL:
      can_fd_custom_parse_entries(payload, payload_len, 5u, can_fd_custom_apply_pos_vel_entry);
      break;

    case CAN_FD_CMD_TAR_POS_CURR:
      can_fd_custom_parse_entries(payload, payload_len, 5u, can_fd_custom_apply_pos_curr_entry);
      break;

    case CAN_FD_CMD_TAR_VEL_CURR:
      can_fd_custom_parse_entries(payload, payload_len, 5u, can_fd_custom_apply_vel_curr_entry);
      break;

    case CAN_FD_CMD_TAR_POS_VEL_CURR:
      can_fd_custom_handle_pos_vel_curr(payload, payload_len);
      break;

    case CAN_FD_CMD_FAULT_RESET:
      result = mc_reset_from_exception();
      reply_payload[0] = (result == MC_OK) ? 1u : 0u;
      can_fd_custom_reply(CAN_FD_CMD_FAULT_RESET, reply_payload, 1u);
      break;

    case CAN_FD_CMD_ENABLE:
      if (payload_len >= 1u)
      {
        if (payload[0] != 0u)
        {
          (void)mc_enable();
        }
        else
        {
          (void)mc_disable();
        }
      }
      break;

    case CAN_FD_CMD_CONTROL_MODE:
      if ((payload_len >= 1u) && !mc_is_enabled())
      {
        mc_set_ctrl_mode(can_fd_custom_wire_mode_to_ctrl(payload[0]));
      }
      break;

    case CAN_FD_CMD_SET_POS_PID:
      if (payload_len >= 6u)
      {
        if (can_fd_custom_require_disabled())
        {
          can_fd_codec_unpack_pid(payload, &m->config.pid.position.kp, &m->config.pid.position.ki,
                                  &m->config.pid.position.kd);
          can_fd_custom_reply_pid(CAN_FD_CMD_GET_POS_PID, &m->config.pid.position);
        }
      }
      break;

    case CAN_FD_CMD_SET_VEL_PID:
      if (payload_len >= 6u)
      {
        if (can_fd_custom_require_disabled())
        {
          can_fd_codec_unpack_pid(payload, &m->config.pid.velocity.kp, &m->config.pid.velocity.ki,
                                  &m->config.pid.velocity.kd);
          can_fd_custom_reply_pid(CAN_FD_CMD_GET_VEL_PID, &m->config.pid.velocity);
        }
      }
      break;

    case CAN_FD_CMD_SET_CUR_PID:
      if (payload_len >= 6u)
      {
        if (can_fd_custom_require_disabled())
        {
          can_fd_codec_unpack_pid(payload, &m->config.pid.iq.kp, &m->config.pid.iq.ki, &m->config.pid.iq.kd);
          can_fd_custom_reply_pid(CAN_FD_CMD_GET_CUR_PID, &m->config.pid.iq);
        }
      }
      break;

    case CAN_FD_CMD_SET_STIFF_DAMP:
      if (payload_len >= 4u)
      {
        if (can_fd_custom_require_disabled())
        {
          m->config.mit.stiffness = can_fd_codec_q12_to_float(can_fd_codec_read_u16_be(&payload[0]));
          m->config.mit.damping = can_fd_codec_q12_to_float(can_fd_codec_read_u16_be(&payload[2]));
          can_fd_codec_write_u16_be(&reply_payload[0], can_fd_codec_float_to_q12(m->config.mit.stiffness));
          can_fd_codec_write_u16_be(&reply_payload[2], can_fd_codec_float_to_q12(m->config.mit.damping));
          can_fd_custom_reply(CAN_FD_CMD_GET_STIFF_DAMP, reply_payload, 4u);
        }
      }
      break;

    case CAN_FD_CMD_SET_PROTECT:
      if (payload_len >= 6u)
      {
        if (can_fd_custom_require_disabled())
        {
          can_fd_codec_unpack_protect(payload, &m->config.limits);
          can_fd_codec_pack_protect(&m->config.limits, reply_payload);
          can_fd_custom_reply(CAN_FD_CMD_GET_PROTECT, reply_payload, 6u);
        }
      }
      break;

    case CAN_FD_CMD_GET_POS_PID:
      can_fd_custom_reply_pid(CAN_FD_CMD_GET_POS_PID, &m->config.pid.position);
      break;

    case CAN_FD_CMD_GET_VEL_PID:
      can_fd_custom_reply_pid(CAN_FD_CMD_GET_VEL_PID, &m->config.pid.velocity);
      break;

    case CAN_FD_CMD_GET_CUR_PID:
      can_fd_custom_reply_pid(CAN_FD_CMD_GET_CUR_PID, &m->config.pid.iq);
      break;

    case CAN_FD_CMD_GET_STIFF_DAMP:
      can_fd_codec_write_u16_be(&reply_payload[0], can_fd_codec_float_to_q12(m->config.mit.stiffness));
      can_fd_codec_write_u16_be(&reply_payload[2], can_fd_codec_float_to_q12(m->config.mit.damping));
      can_fd_custom_reply(CAN_FD_CMD_GET_STIFF_DAMP, reply_payload, 4u);
      break;

    case CAN_FD_CMD_GET_PROTECT:
      can_fd_codec_pack_protect(&m->config.limits, reply_payload);
      can_fd_custom_reply(CAN_FD_CMD_GET_PROTECT, reply_payload, 6u);
      break;

    case CAN_FD_CMD_SET_NODE_ID:
      if ((payload_len >= 2u) && can_fd_custom_require_disabled())
      {
        uint16_t new_id = can_fd_codec_read_u16_be(payload);

        if ((new_id > 0u) && (new_id <= 0x7FEu))
        {
          m->config.comm.can_id = new_id;
          s_node_id = new_id;
          (void)comm_set_node_id_filter(new_id);
          can_fd_codec_write_u16_be(node_id_be, new_id);
          can_fd_custom_reply(CAN_FD_CMD_SET_NODE_ID, node_id_be, 2u);
        }
      }
      break;

    case CAN_FD_CMD_SAVE_PARAMS:
      if (can_fd_custom_require_disabled())
      {
        reply_payload[0] = motor_config_save(&m->config) ? 1u : 0u;
        can_fd_custom_reply(CAN_FD_CMD_SAVE_PARAMS, reply_payload, 1u);
      }
      break;

    case CAN_FD_CMD_RESET_TO_FACTORY:
      if (can_fd_custom_require_disabled())
      {
        reply_payload[0] = motor_config_reset(&m->config) ? 1u : 0u;
        if (reply_payload[0] != 0u)
        {
          s_node_id = m->config.comm.can_id;
        }
        can_fd_custom_reply(CAN_FD_CMD_RESET_TO_FACTORY, reply_payload, 1u);
      }
      break;

    case CAN_FD_CMD_CLEAR_FAULT:
      result = mc_try_recover_from_fault();
      reply_payload[0] = (result == MC_OK) ? 1u : 0u;
      can_fd_custom_reply(CAN_FD_CMD_CLEAR_FAULT, reply_payload, 1u);
      break;

    case CAN_FD_CMD_SET_ORIGIN:
      if (can_fd_custom_require_disabled())
      {
        (void)mc_set_joint_origin();
      }
      break;

    case CAN_FD_CMD_GET_MOTOR_TYPE:
      reply_payload[0] = 0u;
      can_fd_custom_reply(CAN_FD_CMD_GET_MOTOR_TYPE, reply_payload, 1u);
      break;

    case CAN_FD_CMD_SET_REPORT_RATE:
      if (payload_len >= 2u)
      {
        hz = can_fd_codec_read_u16_be(payload);
        if (hz <= CAN_FD_REPORT_HZ_MAX)
        {
          s_report_interval_us = (hz == 0u) ? 0u : (1000000u / (uint32_t)hz);
        }
      }
      break;

    case CAN_FD_CMD_ALIGN_ELECTRIC_ANGLE:
      if (can_fd_custom_require_disabled())
      {
        result = mc_align_electrical();
        reply_payload[0] = (result == MC_OK) ? 1u : 0u;
        can_fd_custom_reply(CAN_FD_CMD_ALIGN_ELECTRIC_ANGLE, reply_payload, 1u);
      }
      break;

    case CAN_FD_CMD_MCU_TEMP_CALIB:
      if ((payload_len >= 2u) && can_fd_custom_require_disabled())
      {
        temp_cal_raw = can_fd_codec_read_i16_be(payload);
        s_mcu_temp_cal_c = (float)temp_cal_raw / 100.0f;
        reply_payload[0] = 1u;
        can_fd_custom_reply(CAN_FD_CMD_MCU_TEMP_CALIB, reply_payload, 1u);
      }
      break;

    case CAN_FD_CMD_KTH71_CAL_START:
      if (!mc_kth71_cal_active())
      {
        reply_payload[0] = (mc_kth71_cal_start() == MC_OK) ? 1u : 0u;
        can_fd_custom_reply(CAN_FD_CMD_KTH71_CAL_START, reply_payload, 1u);
      }
      break;

    case CAN_FD_CMD_KTH71_CAL_STOP:
      reply_payload[0] = (mc_kth71_cal_stop() == MC_OK) ? 1u : 0u;
      can_fd_custom_reply(CAN_FD_CMD_KTH71_CAL_STOP, reply_payload, 1u);
      break;

    case CAN_FD_CMD_KTH71_CAL_TRIGGER:
      reply_payload[0] = (mc_kth71_cal_trigger() == MC_OK) ? 1u : 0u;
      can_fd_custom_reply(CAN_FD_CMD_KTH71_CAL_TRIGGER, reply_payload, 1u);
      break;

    case CAN_FD_CMD_KTH71_CAL_STATUS:
    {
      uint8_t anlc_status = 0u;

      (void)mc_kth71_cal_get_status(&anlc_status);
      reply_payload[0] = anlc_status;
      can_fd_custom_reply(CAN_FD_CMD_KTH71_CAL_STATUS, reply_payload, 1u);
      break;
    }

    default:
      break;
  }
}

static void can_fd_custom_period_report(void)
{
  motor_handle_t *m = motor_get_handler();
  motor_state_t *s = &m->state;
  uint8_t payload[64];
  int16_t pos_raw;
  int16_t vel_raw;
  int16_t iq_ma;
  int16_t id_ma;
  int16_t ia_ma;
  int16_t ib_ma;
  int16_t ic_ma;
  uint8_t status;
  uint8_t error;
  float temp_c;
  uint8_t temp_u8;
  uint8_t vbus_u8;
  fault_t fault_bits;

  /* joint 位置/速度均为控制系，与 ref 字段一一对应 */
  pos_raw = can_fd_codec_rad_to_pos_q7(s->joint.angle_rad);
  vel_raw = can_fd_codec_rad_s_to_vel_q7_rpm(s->joint.velocity_rad_s);
  iq_ma = (int16_t)(s->i_q * 1000.0f);
  id_ma = (int16_t)(s->i_d * 1000.0f);
  ia_ma = (int16_t)(s->phase_current.ampere[0] * 1000.0f);
  ib_ma = (int16_t)(s->phase_current.ampere[1] * 1000.0f);
  ic_ma = (int16_t)(s->phase_current.ampere[2] * 1000.0f);

  status = (uint8_t)(can_fd_custom_ctrl_to_wire_mode(mc_get_ctrl_mode()) & 0x0Fu);
  if (mc_is_enabled())
  {
    status |= 0x10u;
  }

  fault_bits = motor_fault_latched_get(m);
  if (fault_bits == FAULT_NONE)
  {
    fault_bits = motor_fault_status_get(m);
  }
  error = can_fd_codec_fault_to_error(fault_bits);

  temp_c = s->board_temp.value;
  temp_u8 = (uint8_t)(temp_c + 0.5f);

  if (s->vbus < 0.0f)
  {
    s->vbus = 0.0f;
  }
  if (s->vbus > 25.5f)
  {
    vbus_u8 = 255u;
  }
  else
  {
    vbus_u8 = (uint8_t)(s->vbus * 10.0f + 0.5f);
  }

  can_fd_codec_write_i16_be(&payload[0], pos_raw);
  can_fd_codec_write_i16_be(&payload[2], vel_raw);
  can_fd_codec_write_i16_be(&payload[4], iq_ma);
  can_fd_codec_write_i16_be(&payload[6], id_ma);
  can_fd_codec_write_i16_be(&payload[8], ia_ma);
  can_fd_codec_write_i16_be(&payload[10], ib_ma);
  can_fd_codec_write_i16_be(&payload[12], ic_ma);
  payload[14] = status;
  payload[15] = error;
  payload[16] = temp_u8;
  payload[17] = vbus_u8;

  (void)can_fd_custom_send_frame(s_node_id, CAN_FD_CMD_PERIOD_REPORT, payload, 18u);
}

static bool can_fd_custom_init(uint16_t node_id)
{
  s_node_id = node_id;
  s_motor = motor_get_handler();
  s_last_report_us = board_time_us();
  s_tx_cnt = 0u;
  s_report_interval_us = 1000000u / (uint32_t)CAN_FD_REPORT_HZ_DEFAULT;
  s_mcu_temp_cal_c = 0.0f;
  return (node_id <= 0x7FFu);
}

static void can_fd_custom_deinit(void)
{
  s_send = NULL;
}

static void can_fd_custom_on_rx(const comm_frame_t *frame)
{
  uint8_t cmd;
  const uint8_t *payload;
  uint8_t payload_len;

  if (frame == NULL || frame->len < 2u)
  {
    return;
  }

  if (!can_fd_custom_is_for_this_node(frame->id))
  {
    return;
  }

  cmd = frame->data[1];
  payload = &frame->data[2];
  payload_len = (uint8_t)(frame->len - 2u);

  if (frame->id == CAN_FD_IAP_ID)
  {
    if (cmd >= CAN_FD_CMD_IAP_ENTER)
    {
      can_fd_iap_on_rx(frame->id, cmd, payload, payload_len, s_send, s_node_id, &s_tx_cnt);
    }
    return;
  }

  if (cmd >= CAN_FD_CMD_IAP_ENTER)
  {
    can_fd_iap_on_rx(frame->id, cmd, payload, payload_len, s_send, s_node_id, &s_tx_cnt);
    return;
  }

  can_fd_custom_handle_command(cmd, payload, payload_len);
}

static void can_fd_custom_poll(void)
{
  uint32_t now = board_time_us();

  if (s_report_interval_us == 0u)
  {
    return;
  }

  if ((now - s_last_report_us) >= s_report_interval_us)
  {
    s_last_report_us = now;
    can_fd_custom_period_report();
  }
}

static void can_fd_custom_bind_transport(bool (*send_fn)(const comm_frame_t *frame))
{
  s_send = send_fn;
}

static const comm_protocol_ops_t s_can_fd_custom_ops = {
    .name = "can_fd_custom",
    .init = can_fd_custom_init,
    .deinit = can_fd_custom_deinit,
    .on_rx = can_fd_custom_on_rx,
    .poll = can_fd_custom_poll,
    .bind_transport = can_fd_custom_bind_transport,
};

const comm_protocol_ops_t *can_fd_custom_protocol_ops(void)
{
  return &s_can_fd_custom_ops;
}
