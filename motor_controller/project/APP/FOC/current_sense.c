#include "current_sense.h"
#include "board.h"
#include "motor_axis.h"

static void current_sense_offset_cal_step(motor_handle_t *m)
{
    uint32_t i;
    static uint32_t s_cnt;
    static uint32_t s_sum[3];
    static bool s_active;

    if (m == NULL)
    {
        return;
    }

    if (m->current_offset_cal_done)
    {
        s_active = false;
        return;
    }

    if ((!m->current_offset_cal_pending) && (!s_active))
    {
        return;
    }

    if (!s_active)
    {
        s_active = true;
        s_cnt = 0U;
        s_sum[0] = 0U;
        s_sum[1] = 0U;
        s_sum[2] = 0U;
        m->current_offset_cal_pending = false;
        m->current_offset_calibrating = true;
    }

    for (i = 0U; i < 3U; i++)
    {
        s_sum[i] += (uint32_t)m->state.phase_current.adc_raw[i];
    }
    s_cnt++;

    if (s_cnt >= CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT)
    {
        for (i = 0U; i < 3U; i++)
        {
            m->state.phase_current.adc_offset[i] =
                (uint16_t)(s_sum[i] >> CURRENT_OFFSET_CALIBRATION_TIME);
        }
        m->current_offset_calibrating = false;
        m->current_offset_cal_done = true;
        s_active = false;
    }
}

static void current_sense_apply_raw_to_physical(motor_handle_t *m)
{
    uint32_t i;
    const float kv = CURRENT_SENSE_ADC_VOLTS_PER_LSB;
    const float gain = CURRENT_SENSE_SO_AMPS_PER_VOLT;
    motor_state_t *state;

    if (m == NULL)
    {
        return;
    }

    state = &m->state;

    for (i = 0U; i < 3U; i++)
    {
        float v_pin = 0.0f;
        if (!m->current_offset_cal_done)
        {
            v_pin = (float)state->phase_current.adc_raw[i] * kv;
            state->phase_current.ampere[i] = (v_pin - CURRENT_SENSE_SO_BIAS_V) * gain;
        }
        else
        {
            v_pin = (float)((uint16_t)state->phase_current.adc_raw[i] - (uint16_t)state->phase_current.adc_offset[i]) * kv;
            state->phase_current.ampere[i] = v_pin * gain;
        }
    }

    state->board_temp.value =
        (float)state->board_temp.adc_raw * CURRENT_SENSE_ADC_VREF_V / CURRENT_SENSE_ADC_FULL_SCALE;
}

static void current_sense_run_offset_cal_blocking(motor_handle_t *m)
{
    if (m == NULL)
    {
        return;
    }

    motor_axis_prepare_dc_cal(m);

    m->current_offset_cal_pending = true;
    m->current_offset_calibrating = false;
    m->current_offset_cal_done = false;

    while (!m->current_offset_cal_done)
    {
        board_get_phase_current(m);
        current_sense_process_sample(m);
    }

    motor_axis_disarm(m);
}

void current_sense_init(motor_handle_t *m)
{
    uint32_t i;

    if (m == NULL)
    {
        return;
    }

    m->current_offset_cal_pending = true;
    m->current_offset_calibrating = false;
    m->current_offset_cal_done = false;

    for (i = 0U; i < 3U; i++)
    {
        m->state.phase_current.adc_offset[i] = 0U;
    }

    current_sense_run_offset_cal_blocking(m);
}

void current_sense_process_sample(motor_handle_t *m)
{
    current_sense_apply_raw_to_physical(m);
    current_sense_offset_cal_step(m);
}
