#include "mp6540.h"
#include "mp6540_hal_adapt.h"

static void mp6540_set_phase_en(MP6540_Handle_t *hmp6, gate_driver_phase_t phase, bool en)
{
    gpio_port_t port;
    gpio_pin_t pin;

    switch (phase)
    {
        case GATE_DRIVER_PHASE_A:
            port = hmp6->hw.port_ENA;
            pin = hmp6->hw.pin_ENA;
            break;
        case GATE_DRIVER_PHASE_B:
            port = hmp6->hw.port_ENB;
            pin = hmp6->hw.pin_ENB;
            break;
        case GATE_DRIVER_PHASE_C:
            port = hmp6->hw.port_ENC;
            pin = hmp6->hw.pin_ENC;
            break;
        default:
            return;
    }

    MP6540_HW_GPIO_Set(port, pin, en);
}

static void mp6540_set_all_phase_en(MP6540_Handle_t *hmp6, bool en)
{
    mp6540_set_phase_en(hmp6, GATE_DRIVER_PHASE_A, en);
    mp6540_set_phase_en(hmp6, GATE_DRIVER_PHASE_B, en);
    mp6540_set_phase_en(hmp6, GATE_DRIVER_PHASE_C, en);
}

static void mp6540_sleep_enter(MP6540_Handle_t *hmp6)
{
    if (hmp6->is_sleep)
    {
        return;
    }

    mp6540_set_all_phase_en(hmp6, false);
    MP6540_HW_GPIO_Set(hmp6->hw.port_nSLEEP, hmp6->hw.pin_nSLEEP, false);
    hmp6->is_sleep = true;
}

static void mp6540_sleep_exit(MP6540_Handle_t *hmp6)
{
    if (!hmp6->is_sleep)
    {
        return;
    }

    MP6540_HW_GPIO_Set(hmp6->hw.port_nSLEEP, hmp6->hw.pin_nSLEEP, true);
    MP6540_HW_DelayMs(MP6540_WAKEUP_DELAY_MS);
    hmp6->is_sleep = false;
}

static void mp6540_op_enter_inactive_state(void *ctx)
{
    MP6540_Handle_t *hmp6 = (MP6540_Handle_t *)ctx;

    mp6540_set_all_phase_en(hmp6, false);
    mp6540_sleep_enter(hmp6);
    MP6540_HW_DelayMs(MP6540_CAL_DELAY_MS);
}

static void mp6540_op_exit_inactive_state(void *ctx)
{
    MP6540_Handle_t *hmp6 = (MP6540_Handle_t *)ctx;

    mp6540_sleep_exit(hmp6);
    mp6540_set_all_phase_en(hmp6, true);
}

static void mp6540_op_enter_sleep(void *ctx)
{
    mp6540_sleep_enter((MP6540_Handle_t *)ctx);
}

static void mp6540_op_exit_sleep(void *ctx)
{
    mp6540_sleep_exit((MP6540_Handle_t *)ctx);
}

static void mp6540_op_set_all_phase_en(void *ctx, bool en)
{
    mp6540_set_all_phase_en((MP6540_Handle_t *)ctx, en);
}

static bool mp6540_op_read_fault(void *ctx)
{
    MP6540_Handle_t *hmp6 = (MP6540_Handle_t *)ctx;

    return MP6540_HW_GPIO_Read(hmp6->hw.port_FAULT, hmp6->hw.pin_FAULT);
}

static const gate_driver_ops_t s_mp6540_gate_driver_ops = {
    .enter_inactive_state = mp6540_op_enter_inactive_state,
    .exit_inactive_state = mp6540_op_exit_inactive_state,
    .enter_sleep = mp6540_op_enter_sleep,
    .exit_sleep = mp6540_op_exit_sleep,
    .set_all_phase_en = mp6540_op_set_all_phase_en,
    .read_fault = mp6540_op_read_fault,
};

void MP6540_Init(MP6540_Handle_t *hmp6, const gate_driver_hw_cfg_t *hw_cfg)
{
    hmp6->hw = *hw_cfg;
    hmp6->is_sleep = false;
    mp6540_sleep_enter(hmp6);
}

const gate_driver_t *MP6540_AsGateDriver(MP6540_Handle_t *hmp6)
{
    static gate_driver_t s_drv;

    s_drv.ctx = (void *)hmp6;
    s_drv.ops = &s_mp6540_gate_driver_ops;
    return &s_drv;
}

void MP6540_RegisterAsActiveDriver(MP6540_Handle_t *hmp6)
{
    gate_driver_set_active(MP6540_AsGateDriver(hmp6));
}
