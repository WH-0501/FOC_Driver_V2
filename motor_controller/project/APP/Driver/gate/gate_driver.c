#include "gate_driver.h"
#include "mp6540.h"

static const gate_driver_t *s_active_gate_driver;
static MP6540_Handle_t s_mp6540_handle;

void gate_driver_set_active(const gate_driver_t *drv)
{
    s_active_gate_driver = drv;
}

const gate_driver_t *gate_driver_get_active(void)
{
    return s_active_gate_driver;
}

int gate_driver_init(const gate_driver_init_t *init)
{
    if (init == NULL)
    {
        gate_driver_set_active(NULL);
        return GATE_DRIVER_INIT_OK;
    }

    switch (init->type)
    {
        case GATE_DRIVER_TYPE_NONE:
            gate_driver_set_active(NULL);
            return GATE_DRIVER_INIT_OK;

        case GATE_DRIVER_TYPE_MP6540:
            MP6540_Init(&s_mp6540_handle, &init->hw);
            gate_driver_set_active(MP6540_AsGateDriver(&s_mp6540_handle));
            return GATE_DRIVER_INIT_OK;

        case GATE_DRIVER_TYPE_DRV8311:
            /* TODO: 新增 DRV8311 驱动后在此接入 */
            return GATE_DRIVER_INIT_ERR_NOT_IMPL;

        default:
            return GATE_DRIVER_INIT_ERR_PARAM;
    }
}
