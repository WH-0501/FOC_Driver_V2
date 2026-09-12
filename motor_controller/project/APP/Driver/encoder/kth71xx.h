#ifndef __KTH71XX_H
#define __KTH71XX_H

#ifdef __cplusplus
extern "C" {
#endif

	
#define KTH71_READ_ANGLE                        0x00
#define KTH71_READ_REG                          0x11
#define KTH71_WRITE_REG                         0x33


unsigned char KTH71_ReadAngle(unsigned short* pAngle);
void KTH71_WriteRegToMTP(void);
void KTH71_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __KTH71XX_H */
