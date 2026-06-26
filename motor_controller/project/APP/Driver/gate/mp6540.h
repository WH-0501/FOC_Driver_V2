#ifndef __MP6540_H__
#define __MP6540_H__

#include <stdint.h>
#include <stdbool.h>
#include "gate_driver.h"

// ===================== 配置宏（可工程内修改） =====================
#define MP6540_CAL_DELAY_MS      10U     // 校准前放电延时ms
#define MP6540_WAKEUP_DELAY_MS   1U      // nSLEEP唤醒稳定延时

// MP6540 运行实例结构体（一个芯片对应一个实例）
typedef struct
{
    gate_driver_hw_cfg_t hw;            // 通用驱动硬件配置
    bool is_sleep;                      // 芯片是否休眠
} MP6540_Handle_t;

// ===================== 底层硬件适配接口声明（由MCU适配层实现） =====================
// GPIO操作
extern void MP6540_HW_GPIO_Set(gpio_port_t port, gpio_pin_t pin, bool level);
extern bool MP6540_HW_GPIO_Read(gpio_port_t port, gpio_pin_t pin);
// 毫秒级延时
extern void MP6540_HW_DelayMs(uint32_t ms);

// ===================== 上层对外业务API =====================
/**
 * @brief  MP6540实例初始化，默认进入睡眠状态
 * @param  hmp6: MP6540实例句柄
 * @param  hw_cfg: 硬件引脚配置
 */
void MP6540_Init(MP6540_Handle_t *hmp6, const gate_driver_hw_cfg_t *hw_cfg);

/**
 * @brief  芯片进入深度睡眠（nSLEEP拉低，所有驱动/采样运放断电）
 */
void MP6540_EnterSleep(MP6540_Handle_t *hmp6);

/**
 * @brief  芯片退出睡眠唤醒
 */
void MP6540_ExitSleep(MP6540_Handle_t *hmp6);

/**
 * @brief  设置单分相使能
 * @param  phase: gate_driver_phase_t
 * @param  en: true=使能输出，false=该相半桥高阻关闭
 */
void MP6540_SetPhaseEn(MP6540_Handle_t *hmp6, gate_driver_phase_t phase, bool en);

/**
 * @brief  全部三相同时使能/关闭
 */
void MP6540_SetAllPhaseEn(MP6540_Handle_t *hmp6, bool en);

/**
 * @brief  进入非激活态：三相EN关闭→nSLEEP休眠→放电延时
 */
void MP6540_EnterInactiveState(MP6540_Handle_t *hmp6);

/**
 * @brief  退出非激活态：退出休眠并恢复相使能
 */
void MP6540_ExitInactiveState(MP6540_Handle_t *hmp6);

/**
 * @brief  读取FAULT故障引脚电平
 * @retval true:故障触发，false:正常
 */
bool MP6540_ReadFault(MP6540_Handle_t *hmp6);

/**
 * @brief  将 MP6540 实例导出为通用 gate_driver 抽象
 * @param  hmp6: MP6540实例句柄
 * @retval gate_driver_t，可供上层 current_sense 等模块调用
 */
const gate_driver_t *MP6540_AsGateDriver(MP6540_Handle_t *hmp6);

/**
 * @brief  将 MP6540 注册为当前生效 gate driver
 */
void MP6540_RegisterAsActiveDriver(MP6540_Handle_t *hmp6);

#endif /* __MP6540_H__ */
