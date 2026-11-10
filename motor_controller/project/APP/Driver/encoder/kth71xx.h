#ifndef __KTH71XX_H
#define __KTH71XX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 逻辑片选编号，实际 SPI 和引脚由KTH71 驱动映射。 */
typedef enum {
  KTH71_CS_0,
  KTH71_CS_1
} kth71_cs_t;
	
#define KTH71_READ_ANGLE                        0x00
#define KTH71_READ_REG                          0x11
#define KTH71_WRITE_REG                         0x33

#define KTH71_REG_GAINTRIM_DEFAULT                (0xAC)
#define KTH71_REG_CAL_CTRL_DEFAULT                (0x00)
#define KTH71_REG_ANLC_STATUS_DEFAULT             (0x00)
#define KTH71_REG_ANLC_STATUS_DEFAULT             (0x00)

/* 离轴校准值.在轴安装时设置为 0. 默认值为 0xAC */
#define KTH71_REG_GAINTRIM                        (0x11)
/* 非线性校准控制寄存器. 默认值为 0x00 */
#define KTH71_REG_CAL_CTRL                        (0x16)
/* 自校准状态寄存器. 默认值为 0x00(表示未开始校准) */
#define KTH71_REG_ANLC_STATUS                     (0x72)

/** 0x16：bit3=ANLC_EN，bit4=REG_CAL（默认 0x08） */
#define KTH71_ANLC_EN_BIT                         (1u << 3) ///< 自校准效果使能到角度输出
#define KTH71_REG_CAL_BIT                         (1u << 4) ///< 自校准使能. 1: 开始自校准

/** 0x72：ANLC_STATUS[1:0] 位于 bit[5:4]（手册 Figure 26 / §12.2） */
#define KTH71_ANLC_STATUS_MASK                  0x03u
#define KTH71_ANLC_STATUS_SHIFT                 4u
#define KTH71_ANLC_STATUS_IDLE                  0u ///< 自校准未开始
#define KTH71_ANLC_STATUS_RUNNING               1u ///< 自校准进行中
#define KTH71_ANLC_STATUS_FAILED                2u ///< 自校准失败
#define KTH71_ANLC_STATUS_DONE                  3u ///< 自校准完成

/* 当前硬件接 CS0；CS1 留待双轴板确定接线。 */
void KTH71_InitCS(kth71_cs_t cs);
unsigned char KTH71_ReadAngleCS(kth71_cs_t cs, uint16_t *angle);
unsigned char KTH71_ReadAngle(unsigned short* pAngle);
void KTH71_WriteRegToMTP(void);
void KTH71_Init(void);

/* 自校准相关函数 */
int8_t KTH71_StartCalibration(void);
int8_t KTH71_StopCalibration(void);
int8_t KTH71_GetCalibrationStatus(uint8_t *status);

#ifdef __cplusplus
}
#endif

#endif /* __KTH71XX_H */
