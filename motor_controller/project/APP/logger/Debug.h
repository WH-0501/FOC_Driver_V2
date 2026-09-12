#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>


typedef union 
{
  float 	FloatData;
  unsigned char  ByteData[4];
}FLOATtoByet;

typedef struct  
{
  FLOATtoByet Data[7];
}SENDDATA;

void VoFaDisUart(void);


#endif
