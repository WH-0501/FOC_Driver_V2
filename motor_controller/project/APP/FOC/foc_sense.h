#ifndef __FOC_SENSE_H__
#define __FOC_SENSE_H__

#include "datatypes.h"

/** 慢路径：采样并滤波母线电压（约 FOC_VBUS_SAMPLE_FS_HZ，在 mc_poll 中调用）。 */
void foc_sense_poll_vbus(motor_handle_t *m);

/** 电流环周期内更新观测量：编码器 + 相电流 Park 分量（不含母线，避免占用 ISR 时间）。 */
void foc_sense_update(motor_handle_t *m);

#endif /* __FOC_SENSE_H__ */
