#include "mp6540.h"
#include "mp6540_hal_adapt.h"

static void mp6540_enter_inactive_state(void *ctx)
{
    MP6540_EnterInactiveState((MP6540_Handle_t *)ctx);
}

static void mp6540_exit_inactive_state(void *ctx)
{
    MP6540_ExitInactiveState((MP6540_Handle_t *)ctx);
}

static const gate_driver_ops_t s_mp6540_gate_driver_ops = {
    .enter_inactive_state = mp6540_enter_inactive_state,
    .exit_inactive_state = mp6540_exit_inactive_state,
};

void MP6540_Init(MP6540_Handle_t *hmp6, const gate_driver_hw_cfg_t *hw_cfg)
{
    // 拷贝硬件配置
    hmp6->hw = *hw_cfg;
    // 上电默认拉低nSLEEP进入睡眠
    MP6540_EnterSleep(hmp6);
}

void MP6540_EnterSleep(MP6540_Handle_t *hmp6)
{
    if(hmp6->is_sleep) return;
    // 先关分相使能
    MP6540_SetAllPhaseEn(hmp6, false);
    // 拉低nSLEEP休眠
    MP6540_HW_GPIO_Set(hmp6->hw.port_nSLEEP, hmp6->hw.pin_nSLEEP, false);
    hmp6->is_sleep = true;
}

void MP6540_ExitSleep(MP6540_Handle_t *hmp6)
{
    if(!hmp6->is_sleep) return;
    // 拉高唤醒
    MP6540_HW_GPIO_Set(hmp6->hw.port_nSLEEP, hmp6->hw.pin_nSLEEP, true);
    MP6540_HW_DelayMs(MP6540_WAKEUP_DELAY_MS);
    hmp6->is_sleep = false;
}

void MP6540_SetPhaseEn(MP6540_Handle_t *hmp6, gate_driver_phase_t phase, bool en)
{
    gpio_port_t port;
    gpio_pin_t pin;
    switch(phase)
    {
        case GATE_DRIVER_PHASE_A: port = hmp6->hw.port_ENA; pin = hmp6->hw.pin_ENA; break;
        case GATE_DRIVER_PHASE_B: port = hmp6->hw.port_ENB; pin = hmp6->hw.pin_ENB; break;
        case GATE_DRIVER_PHASE_C: port = hmp6->hw.port_ENC; pin = hmp6->hw.pin_ENC; break;
        default: return;
    }
    MP6540_HW_GPIO_Set(port, pin, en);
}

void MP6540_SetAllPhaseEn(MP6540_Handle_t *hmp6, bool en)
{
    MP6540_SetPhaseEn(hmp6, GATE_DRIVER_PHASE_A, en);
    MP6540_SetPhaseEn(hmp6, GATE_DRIVER_PHASE_B, en);
    MP6540_SetPhaseEn(hmp6, GATE_DRIVER_PHASE_C, en);
}

void MP6540_EnterInactiveState(MP6540_Handle_t *hmp6)
{
    // 关闭三相分相使能
    MP6540_SetAllPhaseEn(hmp6, false);
    // 进入MP6540深度休眠（关内部采样运放/电荷泵）
    MP6540_EnterSleep(hmp6);
    // 延时等待母线、绕组电容放电
    MP6540_HW_DelayMs(MP6540_CAL_DELAY_MS);
}

void MP6540_ExitInactiveState(MP6540_Handle_t *hmp6)
{
    // 校准完成后唤醒芯片；外部自行控制是否开启PWM/分相使能
    MP6540_ExitSleep(hmp6);
    // 恢复三相分相使能
    MP6540_SetAllPhaseEn(hmp6, true);
}

bool MP6540_ReadFault(MP6540_Handle_t *hmp6)
{
    return MP6540_HW_GPIO_Read(hmp6->hw.port_FAULT, hmp6->hw.pin_FAULT);
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
