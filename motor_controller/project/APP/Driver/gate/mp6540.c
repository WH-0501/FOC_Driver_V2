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

/**
 * @brief 设置所有相位使能状态
 *        1. 设置相位A使能状态
 *        2. 设置相位B使能状态
 *        3. 设置相位C使能状态
 * @param hmp6 
 * @param en 使能状态
 */
static void mp6540_set_all_phase_en(MP6540_Handle_t *hmp6, bool en)
{
    mp6540_set_phase_en(hmp6, GATE_DRIVER_PHASE_A, en);
    mp6540_set_phase_en(hmp6, GATE_DRIVER_PHASE_B, en);
    mp6540_set_phase_en(hmp6, GATE_DRIVER_PHASE_C, en);
}

/**
 * @brief 进入休眠状态
 *        1. 如果已经进入休眠状态，则返回
 *        2. 所有相位使能关闭
 *        3. 设置休眠引脚为低电平
 *        4. 设置休眠状态为true
 * @param hmp6 
 */
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

/**
 * @brief 退出休眠状态
 *        1. 如果已经退出休眠状态，则返回
 *        2. 设置休眠引脚为高电平
 *        3. 延时
 *        4. 设置休眠状态为false
 * @param hmp6 
 */
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

/**
 * @brief 进入非活动状态
 *        1. 所有相位使能关闭
 *        2. 进入休眠状态
 *        3. 延时
 * @param ctx 
 */
static void mp6540_op_enter_inactive_state(void *ctx)
{
    MP6540_Handle_t *hmp6 = (MP6540_Handle_t *)ctx;

    mp6540_set_all_phase_en(hmp6, false);
    mp6540_sleep_enter(hmp6);
    MP6540_HW_DelayMs(MP6540_CAL_DELAY_MS);
}

/**
 * @brief 退出非活动状态
 *        1. 退出休眠状态
 *        2. 所有相位使能开启
 * @param ctx 
 * @return void 
 */
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

/**
 * @brief 读取故障状态
 *        1. 读取故障引脚电平
 *        2. 如果故障引脚电平为高电平，则返回 true
 *        3. 否则返回 false
 * 
 * @param ctx: 电机驱动器句柄
 * @return true: 正常
 * @return false: 故障
 */
static bool mp6540_op_read_fault(void *ctx)
{
    MP6540_Handle_t *hmp6 = (MP6540_Handle_t *)ctx;

    return !MP6540_HW_GPIO_Read(hmp6->hw.port_FAULT, hmp6->hw.pin_FAULT);
}

static const gate_driver_ops_t s_mp6540_gate_driver_ops = {
    .enter_inactive_state = mp6540_op_enter_inactive_state,
    .exit_inactive_state = mp6540_op_exit_inactive_state,
    .enter_sleep = mp6540_op_enter_sleep,
    .exit_sleep = mp6540_op_exit_sleep,
    .set_all_phase_en = mp6540_op_set_all_phase_en,
    .read_fault = mp6540_op_read_fault,
};

/**
 * @brief 初始化 MP6540 电机驱动器
 *        1. 设置硬件配置
 *        2. 设置休眠状态
 * @param hmp6 电机驱动器句柄
 * @param hw_cfg 硬件配置
 */
void MP6540_Init(MP6540_Handle_t *hmp6, const gate_driver_hw_cfg_t *hw_cfg)
{
    hmp6->hw = *hw_cfg;
    hmp6->is_sleep = false;
    mp6540_sleep_enter(hmp6);
}

const gate_driver_t *MP6540_AsGateDriver(MP6540_Handle_t *hmp6)
{
    if (hmp6 == NULL)
    {
        return NULL;
    }

    hmp6->gate.ctx = (void *)hmp6;
    hmp6->gate.ops = &s_mp6540_gate_driver_ops;
    return &hmp6->gate;
}
