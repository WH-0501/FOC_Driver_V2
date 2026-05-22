/**
 * @file kth711x.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __KTH711x_H__
#define __KTH711x_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "datatypes.h"
	
#define KTH71_READ_ANGLE                        0x00
#define KTH71_READ_REG                          0x11
#define KTH71_WRITE_REG                         0x33

unsigned char KTH71_ReadAngle(unsigned short* pAngle);
void KTH71_WriteRegToMTP(void);
void KTH71_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __KTH711x_H__ */