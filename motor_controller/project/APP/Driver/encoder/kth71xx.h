/**
 * @file kth71xx.h
 * @brief KTH71 磁性编码器芯片驱动（SPI 读角/CRC）。上层 unwrap 见 APP/FOC/encoder。
 */

#ifndef __KTH711x_H__
#define __KTH711x_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "datatypes.h"

/**
 * 旧版只有一个磁编. CS 为 PB6
 * 新版有两个磁编：CS 分别为 PA4 和 PA15
 * 
 */
typedef enum
{
  KTH71_CS_0 = 0, /**< GPIOB PIN6 */
  KTH71_CS_1 = 1, /**< GPIOB PIN7 */
  KTH71_CS_2 = 2, /**< GPIOB PIN9 */
  KTH71_CS_COUNT
} kth71_cs_select_t;

#define KTH71_READ_ANGLE                        0x00
#define KTH71_READ_REG                          0x11
#define KTH71_WRITE_REG                         0x33

unsigned char KTH71_ReadAngle(unsigned short *pAngle);
unsigned char KTH71_ReadAngleCs(kth71_cs_select_t cs, unsigned short *pAngle);
void KTH71_WriteRegToMTP(void);
void KTH71_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __KTH71xx_H__ */
