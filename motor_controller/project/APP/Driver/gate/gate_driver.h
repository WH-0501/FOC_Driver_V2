#ifndef __GATE_DRIVER_H__
#define __GATE_DRIVER_H__

#include <stddef.h>
#include <stdbool.h>
#include "../../compiler_port.h"
#include "../../../Board/gate_hw_binding.h"

typedef gate_hw_binding_t gate_driver_hw_cfg_t;

typedef struct
{
    void (*enter_inactive_state)(void *ctx);
    void (*exit_inactive_state)(void *ctx);
    void (*enter_sleep)(void *ctx);
    void (*exit_sleep)(void *ctx);
    void (*set_all_phase_en)(void *ctx, bool en);
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
    GATE_DRIVER_TYPE_DRV8311 = 2,
} gate_driver_type_t;

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

void gate_driver_set_active(const gate_driver_t *drv);
const gate_driver_t *gate_driver_get_active(void);
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

APP_STATIC_INLINE void gate_driver_enter_sleep(const gate_driver_t *drv)
{
    if ((drv != NULL) && (drv->ops != NULL) &&
        (drv->ops->enter_sleep != NULL))
    {
        drv->ops->enter_sleep(drv->ctx);
    }
}

APP_STATIC_INLINE void gate_driver_exit_sleep(const gate_driver_t *drv)
{
    if ((drv != NULL) && (drv->ops != NULL) &&
        (drv->ops->exit_sleep != NULL))
    {
        drv->ops->exit_sleep(drv->ctx);
    }
}

APP_STATIC_INLINE void gate_driver_set_all_phase_en(const gate_driver_t *drv, bool en)
{
    if ((drv != NULL) && (drv->ops != NULL) &&
        (drv->ops->set_all_phase_en != NULL))
    {
        drv->ops->set_all_phase_en(drv->ctx, en);
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
