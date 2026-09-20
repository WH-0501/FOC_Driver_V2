#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>


typedef union 
{
  float 	FloatData;
  unsigned char  ByteData[4];
}FLOATtoByet;

#define VOFA_CHANNEL_COUNT 12u

typedef struct
{
  FLOATtoByet Data[VOFA_CHANNEL_COUNT];
} SENDDATA;

void VoFaDisUart(void);


#endif
