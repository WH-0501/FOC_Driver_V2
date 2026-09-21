#include "mc_interface.h"
#include "foc.h"
#include "foc_calibration.h"
#include "foc_fsm.h"
#include "foc_fault.h"
#include "foc_pid.h"
#include "foc_sense.h"
#include "board.h"
#include "motor.h"
#include "control_mode.h"
#include "error_types.h"
#include "encoder/kth71xx.h"
#include "dwt_profile_delay.h"

#ifndef MC_KTH71_CAL_STATUS_POLL_MS
#define MC_KTH71_CAL_STATUS_POLL_MS (500u)
#endif

typedef enum
{
  MC_KTH71_CAL_PHASE_IDLE = 0,
  MC_KTH71_CAL_PHASE_SPINUP,
  MC_KTH71_CAL_PHASE_ANLC,
} mc_kth71_cal_phase_t;

static struct
{
  mc_kth71_cal_phase_t phase;
  uint8_t anlc_status;
  uint8_t anlc_status_valid;
  uint32_t last_anlc_poll_us;
} s_kth71_cal = {
  .phase = MC_KTH71_CAL_PHASE_IDLE,
  .anlc_status = KTH71_ANLC_STATUS_IDLE,
  .anlc_status_valid = 1u,
  .last_anlc_poll_us = 0u,
};

static void mc_kth71_cal_reset(void)
{
  s_kth71_cal.phase = MC_KTH71_CAL_PHASE_IDLE;
  s_kth71_cal.anlc_status = KTH71_ANLC_STATUS_IDLE;
  s_kth71_cal.anlc_status_valid = 1u;
  s_kth71_cal.last_anlc_poll_us = 0u;
}

static error_t mc_platform_init(void)
{
  board_init();
  return ERR_NONE;
}

static error_t mc_runtime_start(void)
{
  board_register_current_loop_callback(mc_current_loop_isr);
  if (current_hw_init() != ERR_NONE)
  {
    return ERR_SENSOR;
  }
  return ERR_NONE;
}

error_t mc_init(void)
{
  error_t ret;
  motor_handle_t *m;
  motor_config_t cfg;

  /*
   * 必须先载入独立 cfg，再 foc_init 拷贝进 m->config。
   * 不可 motor_config_init(&m->config) 后调 foc_init(m,&m->config)：
   * foc_init 开头 memset(m,0) 会清零刚载入的 config，且 *config 读到的也是全 0。
   */
  (void)motor_config_init(&cfg);
  cfg.param.theta_elec_offset_rad = 3.19311523f;
  cfg.param.direction = DIR_CW;

  ret = mc_platform_init();
  if (ret != ERR_NONE)
  {
    return ret;
  }

  if (motor_hw_init() != ERR_NONE)
  {
    return ERR_FAIL;
  }

  m = motor_get_handler();
  foc_init(m, &cfg);

  ret = mc_runtime_start();
  if (ret != ERR_NONE)
  {
    return ret;
  }

  return ERR_NONE;
}

void mc_poll(void)
{
  motor_handle_t *m = motor_get_handler();

  foc_sense_poll_vbus(m);
  mc_kth71_cal_service();
  motor_fault_protect(m);
}

void mc_current_loop_isr(void)
{
  foc_control_loop(motor_get_handler());
}

void mc_set_ctrl_mode(control_mode_t mode)
{
  motor_handle_t *m = motor_get_handler();

  m->ctrl_mode = mode;

  if ((mode == CTRL_MODE_TORQUE) || (mode == CTRL_MODE_SPEED) ||
      (mode == CTRL_MODE_CST) || (mode == CTRL_MODE_CSV))
  {
    (void)foc_pid_clear(&m->id_pid);
    (void)foc_pid_clear(&m->iq_pid);
  }

  if (mode == CTRL_MODE_IDLE)
  {
    mc_disable();
  }
}

error_t mc_enable(void)
{
  motor_handle_t *m = motor_get_handler();

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return ERR_FAULT_LATCHED;
  }

  if (!m->current_offset_cal_done)
  {
    return ERR_OFFSET_CAL;
  }

  if (!control_mode_needs_pwm(m->ctrl_mode))
  {
    return ERR_CTRL_MODE;
  }

  if (m->fsm == STATE_RUNNING)
  {
    return ERR_NONE;
  }

  if (foc_fsm_is_calibrating(m->fsm))
  {
    return ERR_CALIBRATING;
  }

  if (m->fsm != STATE_IDLE)
  {
    return ERR_STATE;
  }

  foc_fsm_request_running(m);
  return (m->fsm == STATE_RUNNING) ? ERR_NONE : ERR_STATE;
}

static fsm_state_t mc_cal_type_to_fsm_entry(mc_calibration_type_t type)
{
  switch (type)
  {
    case MC_CAL_ENCODER:
      return STATE_ENCODER_CALIBRATION;
    case MC_CAL_RSLS:
      return STATE_RSLS_CALIBRATION;
    case MC_CAL_FLUX:
      return STATE_FLUX_CALIBRATION;
    case MC_CAL_ELECTRICAL_ALIGN:
      return STATE_ELECTRICAL_ALIGNMENT;
    case MC_CAL_ANTICOGGING:
      return STATE_ANTICOGGING;
    case MC_CAL_SEQUENCE:
      return STATE_ENCODER_CALIBRATION;
    default:
      return STATE_MAX;
  }
}

static mc_calibration_type_t mc_fsm_state_to_cal_type(fsm_state_t state)
{
  switch (state)
  {
    case STATE_ENCODER_CALIBRATION:
      return MC_CAL_ENCODER;
    case STATE_RSLS_CALIBRATION:
      return MC_CAL_RSLS;
    case STATE_FLUX_CALIBRATION:
      return MC_CAL_FLUX;
    case STATE_ELECTRICAL_ALIGNMENT:
      return MC_CAL_ELECTRICAL_ALIGN;
    case STATE_ANTICOGGING:
      return MC_CAL_ANTICOGGING;
    default:
      return MC_CAL_NONE;
  }
}

error_t mc_start_calibration(mc_calibration_type_t type)
{
  motor_handle_t *m = motor_get_handler();
  fsm_state_t entry;

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return ERR_FAULT_LATCHED;
  }

  if (m->fsm != STATE_IDLE)
  {
    return ERR_STATE;
  }

  entry = mc_cal_type_to_fsm_entry(type);
  if (entry >= STATE_MAX)
  {
    return ERR_CAL_TYPE;
  }

  foc_fsm_request_calibration(m, entry);
  return foc_fsm_is_calibrating(m->fsm) ? ERR_NONE : ERR_STATE;
}

void mc_stop_calibration(void)
{
  motor_handle_t *m = motor_get_handler();

  if (!foc_fsm_is_calibrating(m->fsm))
  {
    return;
  }

  if (m->fsm != STATE_FAULT)
  {
    foc_fsm_next_state(m, STATE_IDLE);
  }
}

bool mc_is_calibrating(void)
{
  return foc_fsm_is_calibrating(motor_get_handler()->fsm);
}

mc_calibration_type_t mc_get_calibration_type(void)
{
  return mc_fsm_state_to_cal_type(motor_get_handler()->fsm);
}

error_t mc_disable(void)
{
  motor_handle_t *m = motor_get_handler();

  if (s_kth71_cal.phase != MC_KTH71_CAL_PHASE_IDLE)
  {
    if (s_kth71_cal.phase == MC_KTH71_CAL_PHASE_ANLC)
    {
      (void)KTH71_StopCalibration();
    }
    mc_kth71_cal_reset();
  }

  if (foc_fsm_is_calibrating(m->fsm))
  {
    mc_stop_calibration();
    return ERR_CALIBRATING;
  }

  if ((m->fsm != STATE_FAULT) || !motor_fault_is_latched(m))
  {
    foc_fsm_next_state(m, STATE_IDLE);
    foc_fsm_loop(m);
  }

  motor_disarm(m);
  m->out.pwm_enable = 0u;
  return ERR_NONE;
}

bool mc_is_enabled(void)
{
  motor_handle_t *m = motor_get_handler();

  return (m->fsm == STATE_RUNNING) && motor_is_armed(m);
}

control_mode_t mc_get_ctrl_mode(void)
{
  return motor_get_handler()->ctrl_mode;
}

fsm_state_t mc_get_fsm_state(void)
{
  return foc_fsm_get_state(motor_get_handler());
}

error_t mc_try_recover_from_fault(void)
{
  motor_handle_t *m = motor_get_handler();

  if (!motor_fault_is_latched(m))
  {
    return ERR_NO_FAULT;
  }

  motor_fault_clear_latched(m, (fault_t)0xFFFFFFFFu);
  motor_fault_clear_status(m, (fault_t)0xFFFFFFFFu);
  foc_fsm_next_state(m, STATE_IDLE);
  return ERR_NONE;
}

error_t mc_reset_from_exception(void)
{
  motor_handle_t *m = motor_get_handler();

  if (m == NULL)
  {
    return ERR_NULL;
  }

  if (foc_fsm_is_calibrating(m->fsm))
  {
    mc_stop_calibration();
  }

  motor_shutdown(m);
  motor_disarm(m);
  m->out.pwm_enable = 0u;

  motor_fault_clear_latched(m, (fault_t)0xFFFFFFFFu);
  motor_fault_clear_status(m, (fault_t)0xFFFFFFFFu);

  (void)mc_disable();

  (void)foc_pid_clear(&m->id_pid);
  (void)foc_pid_clear(&m->iq_pid);
  (void)foc_pid_clear(&m->velocity_pid);
  (void)foc_pid_clear(&m->position_pid);

  m->ref.id = 0.f;
  m->ref.iq = 0.f;
  m->ref.torque_nm = 0.f;
  m->ref.speed_rad_s = 0.f;
  m->ref.position_rad = m->state.joint.position_rad;

  return ERR_NONE;
}

error_t mc_set_joint_origin(void)
{
  motor_handle_t *m = motor_get_handler();

  if (mc_is_enabled())
  {
    return ERR_STATE;
  }

  {
    float dir_f = (m->config.param.direction == DIR_CCW) ? -1.0f : 1.0f;

    /* position 为控制系；theta_offset 存传感器系零偏：zero += position * dir */
    m->config.param.theta_offset_rad += m->state.shaft.position_rad * dir_f;
  }
  return ERR_NONE;
}



error_t mc_align_electrical(cal_elec_align_method_t method)
{
  motor_handle_t *m = motor_get_handler();
  
  if (m == NULL) return ERR_PARAM;

  if (mc_is_enabled())
  {
    (void)mc_disable();
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return ERR_FAULT_LATCHED;
  }

  if (m->fsm != STATE_IDLE) return ERR_STATE;

  return cal_elec_align_ex(m, method, NULL);
}

bool mc_kth71_cal_active(void)
{
  return s_kth71_cal.phase != MC_KTH71_CAL_PHASE_IDLE;
}

static void mc_kth71_cal_on_status_update(uint8_t status)
{
  s_kth71_cal.anlc_status = status;
  s_kth71_cal.anlc_status_valid = 1u;

  if ((status == KTH71_ANLC_STATUS_DONE) || (status == KTH71_ANLC_STATUS_FAILED))
  {
    (void)KTH71_StopCalibration();
  }
}

static bool mc_kth71_cal_refresh_status(uint32_t now_us, uint32_t min_interval_us)
{
  uint8_t status;
  uint32_t elapsed_us;

  if (s_kth71_cal.phase != MC_KTH71_CAL_PHASE_ANLC)
  {
    s_kth71_cal.anlc_status = 0x66;
    return false;
  }

  if (s_kth71_cal.last_anlc_poll_us != 0u)
  {
    elapsed_us = now_us - s_kth71_cal.last_anlc_poll_us;
    if (elapsed_us < min_interval_us)
    {
      return false;
    }
  }

  s_kth71_cal.last_anlc_poll_us = now_us;

  if (KTH71_GetCalibrationStatus(&status) != 0)
  {
    s_kth71_cal.anlc_status = 0x77;
    return false;
  }

  mc_kth71_cal_on_status_update(status);
  return true;
}

void mc_kth71_cal_service(void)
{
  (void)mc_kth71_cal_refresh_status(
      dwt_get_ticks_us(),
      MC_KTH71_CAL_STATUS_POLL_MS * 1000u);
}

error_t mc_kth71_cal_start(void)
{
  motor_handle_t *m = motor_get_handler();

  if (m == NULL)
  {
    return ERR_NULL;
  }

  if (s_kth71_cal.phase != MC_KTH71_CAL_PHASE_IDLE)
  {
    return ERR_CALIBRATING;
  }

  if (motor_fault_is_latched(m) || (m->fsm == STATE_FAULT))
  {
    return ERR_FAULT_LATCHED;
  }

  s_kth71_cal.anlc_status = KTH71_ANLC_STATUS_IDLE;
  s_kth71_cal.anlc_status_valid = 1u;
  s_kth71_cal.last_anlc_poll_us = 0u;
  s_kth71_cal.phase = MC_KTH71_CAL_PHASE_SPINUP;
  return ERR_NONE;
}

error_t mc_kth71_cal_trigger(void)
{
  if (s_kth71_cal.phase != MC_KTH71_CAL_PHASE_SPINUP)
  {
    return ERR_STATE;
  }

  if (KTH71_StartCalibration() != 0)
  {
    return ERR_SENSOR;
  }

  s_kth71_cal.phase = MC_KTH71_CAL_PHASE_ANLC;
  s_kth71_cal.anlc_status = KTH71_ANLC_STATUS_RUNNING;
  s_kth71_cal.anlc_status_valid = 1u;
  s_kth71_cal.last_anlc_poll_us = 0u;

  if (KTH71_GetCalibrationStatus(&s_kth71_cal.anlc_status) == 0)
  {
    s_kth71_cal.anlc_status_valid = 1u;
  }

  return ERR_NONE;
}

error_t mc_kth71_cal_stop(void)
{
  int8_t chip_ret = 0;

  if (s_kth71_cal.phase != MC_KTH71_CAL_PHASE_IDLE)
  {
    if (s_kth71_cal.phase == MC_KTH71_CAL_PHASE_ANLC)
    {
      chip_ret = KTH71_StopCalibration();
    }
    mc_kth71_cal_reset();
  }

  return (chip_ret == 0) ? ERR_NONE : ERR_SENSOR;
}

error_t mc_kth71_cal_get_status(uint8_t *status)
{
  if (status == NULL)
  {
    return ERR_NULL;
  }

  *status = s_kth71_cal.anlc_status;
  return ERR_NONE;
}
