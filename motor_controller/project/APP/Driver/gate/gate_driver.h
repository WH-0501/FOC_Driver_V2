#ifndef __GATE_DRIVER_H__
#define __GATE_DRIVER_H__

#include <stddef.h>
#include <stdbool.h>
#include "../../compiler_port.h"
#include "../../../Board/platform_gpio_types.h"

typedef struct
{
    void (*enter_inactive_state)(void *ctx); /* 关相使能并进入休眠/高阻等非激活态 */
    void (*exit_inactive_state)(void *ctx);  /* 退出非激活态并恢复可运行状态 */
    bool (*read_fault)(void *ctx);
} gate_driver_ops_t;

typedef struct
{
    void *ctx;
    const gate_driver_ops_t *ops;
} gate_driver_t;

typedef enum
{
    GATE_DRIVER_PHASE_A = 0,
    GATE_DRIVER_PHASE_B,
    GATE_DRIVER_PHASE_C,
    GATE_DRIVER_PHASE_COUNT
} gate_driver_phase_t;

typedef enum
{
    GATE_DRIVER_TYPE_NONE = 0,
    GATE_DRIVER_TYPE_MP6540 = 1,
    GATE_DRIVER_TYPE_DRV8311 = 2, /* 预留 */
} gate_driver_type_t;

/*
 * 通用 gate driver 硬件配置（上层统一传递，不直接绑定具体驱动结构体）。
 * 不同驱动按需使用其中字段；未使用字段可置 0。
 */
typedef struct
{
    gpio_port_t port_nSLEEP;
    gpio_pin_t pin_nSLEEP;
    gpio_port_t port_ENA;
    gpio_pin_t pin_ENA;
    gpio_port_t port_ENB;
    gpio_pin_t pin_ENB;
    gpio_port_t port_ENC;
    gpio_pin_t pin_ENC;
    gpio_port_t port_FAULT;
    gpio_pin_t pin_FAULT;
} gate_driver_hw_cfg_t;

typedef struct
{
    gate_driver_type_t type;
    gate_driver_hw_cfg_t hw;
} gate_driver_init_t;

enum
{
    GATE_DRIVER_INIT_OK = 0,
    GATE_DRIVER_INIT_ERR_PARAM = -1,
    GATE_DRIVER_INIT_ERR_NOT_IMPL = -2,
};

/**
 * @brief 注册当前生效的功率驱动实例
 */
void gate_driver_set_active(const gate_driver_t *drv);

/**
 * @brief 获取当前生效的功率驱动实例；未注册时返回 NULL
 */
const gate_driver_t *gate_driver_get_active(void);

/**
 * @brief 根据配置初始化并注册当前生效驱动
 * @param init 配置参数；传 NULL 等价于清空当前驱动
 * @retval ERR_NONE 成功
 */
int gate_driver_init(const gate_driver_init_t *init);

APP_STATIC_INLINE void gate_driver_enter_inactive_state(const gate_driver_t *drv)
{
    if ((drv != NULL) && (drv->ops != NULL) &&
        (drv->ops->enter_inactive_state != NULL))
    {
        drv->ops->enter_inactive_state(drv->ctx);
    }
}

APP_STATIC_INLINE void gate_driver_exit_inactive_state(const gate_driver_t *drv)
{
    if ((drv != NULL) && (drv->ops != NULL) &&
        (drv->ops->exit_inactive_state != NULL))
    {
        drv->ops->exit_inactive_state(drv->ctx);
    }
}

APP_STATIC_INLINE bool gate_driver_read_fault(const gate_driver_t *drv)
{
    if ((drv != NULL) && (drv->ops != NULL) &&
        (drv->ops->read_fault != NULL))
    {
        return drv->ops->read_fault(drv->ctx);
    }
    return false;
}

#endif /* __GATE_DRIVER_H__ */
