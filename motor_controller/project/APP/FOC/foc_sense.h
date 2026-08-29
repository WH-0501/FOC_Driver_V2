#ifndef __FOC_SENSE_H__
#define __FOC_SENSE_H__

#include "datatypes.h"

/** 电流环周期内更新全部观测量：编码器（shaft/joint）+ 相电流 Park 分量 */
void foc_sense_update(motor_handle_t *m);

#endif /* __FOC_SENSE_H__ */
