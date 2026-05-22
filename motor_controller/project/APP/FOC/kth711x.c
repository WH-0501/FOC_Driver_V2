#include "kth71xx.h"
#include "wk_spi.h"
#include "wk_system.h"


#define SPI_SOFT_CTRL   1


#if SPI_SOFT_CTRL
// 设置GPIOB为输出模式
#define PB_SET_OUTPUT(pin) do { \
    uint32_t shift = ((pin) * 2); \
    GPIOB->cfgr = (GPIOB->cfgr & ~(0x3U << shift)) | (0x1U << shift); \
    GPIOB->omode &= ~(0x1U << (pin)); \
} while(0)

// 设置GPIOB为输入模式
#define PB_SET_INPUT(pin) do { \
    uint32_t shift = ((pin) * 2); \
    GPIOB->cfgr &= ~(0x3U << shift); \
    GPIOB->pull = (GPIOB->pull & ~(0x3U << shift)) | (0x1U << shift); \
} while(0)

#define SCK_SET			 			gpio_bits_set(GPIOB, GPIO_PINS_8)
#define SCK_CLR			 			gpio_bits_reset(GPIOB, GPIO_PINS_8)

#define READ_MISO	   	 		gpio_input_data_bit_read(GPIOB, GPIO_PINS_5)

#define SET_MISO_INPUT		PB_SET_INPUT(GPIO_PINS_5)

#define SET_MISO_OUTPUT		PB_SET_OUTPUT(GPIO_PINS_5)

#define MISO_SET		    	gpio_bits_set(GPIOB, GPIO_PINS_5)
#define MISO_CLR		    	gpio_bits_reset(GPIOB, GPIO_PINS_5)
#endif

#define CSN_SET		   			gpio_bits_set(GPIOB, GPIO_PINS_6)
#define CSN_CLR		   			gpio_bits_reset(GPIOB, GPIO_PINS_6)


/**
  * @brief  CRC8 Table
  */
uint8_t CRC8Table[256]={
0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

static unsigned char KTH71_IsCRCOK(unsigned char* pbuf, unsigned char buflen);
static void KTH71_SPI_TransmitReceive(unsigned char *pTxData, unsigned char *pRxData, unsigned char txLen, unsigned char rxLen);
void delay_us(unsigned int us) 
{
    while(us--) 
		{
			;
			;
		}
}
// 发送一个字节，等待接收
void spi_3wire_send_byte(unsigned char data)
{
#if SPI_SOFT_CTRL
    SET_MISO_OUTPUT;  // 设置为输出模式
    
    // 发送8位数据，最高位在前
    for(unsigned char i = 0; i < 8; i++)
    {
        // 发送数据，在第一个数据之前
        if(data & 0x80)  // 最高位
            MISO_SET;
        else
            MISO_CLR;
        
        data <<= 1;  // 移位，准备发送下一位
        
        // 第一个数据接收，CPHA=1，在第一个数据接收之后发送
        SCK_CLR;  // �����½���
        
        // 等待
        delay_us(1);						//5
        
        // 第二个数据接收，CPHA=1，在第二个数据接收之后发送
        SCK_SET;  // 发送数据，最高位在前
        
        // 等待
        delay_us(1);					//5
    }
    
    // 设置为输入模式
    SET_MISO_INPUT;
#else

    spi_half_duplex_direction_set(SPI2, SPI_HALF_DUPLEX_DIRECTION_TX);
    
    
    spi_i2s_data_transmit(SPI2, data);
    while(spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET);
    
 #endif

    
}
/**
  * @brief  Reads the angle and returns the CRC check result.
  * @param  pAngle pointer to angle.
  * @retval If the CRC check succeeds, return 1. 
            If the CRC check fails, return 0.
  */
unsigned char KTH71_ReadAngle(unsigned short *pAngle)
{
    unsigned char crcflag = 0;
    unsigned char dataSend = KTH71_READ_ANGLE;
    unsigned char dataBack[3];

    KTH71_SPI_TransmitReceive(&dataSend, dataBack, 1, 3);
    crcflag = KTH71_IsCRCOK(dataBack, 3);
    *pAngle = (dataBack[0] << 8) | dataBack[1];
    return crcflag;
}

/**
  * @brief  Reads the register value via the address parameter and returns the CRC check result.
  * @param  Register address.
  * @param  pRegValue pointer to register value.
  * @retval If the CRC check succeeds, return 1. 
            If the CRC check fails, return 0.
  */
unsigned char KTH71_ReadReg(unsigned char addr, unsigned char *pRegValue)
{
    unsigned char crcflag = 0;
    unsigned char dataSend[2] = {KTH71_READ_REG, addr};
    unsigned char dataBack[2];

    KTH71_SPI_TransmitReceive(dataSend, dataBack, 2, 2);
    crcflag = KTH71_IsCRCOK(dataBack, 2);
    *pRegValue = dataBack[0];
    return crcflag;
}

/**
  * @brief  Writes the register.
  * @param  Register address.
  * @param  The value you want to write to the register.
  * @retval The value of the newly written register
  */
unsigned char KTH71_WriteReg(unsigned char addr, unsigned char data)
{
    unsigned char dataSend[3] = {KTH71_WRITE_REG, addr, data};
    unsigned char dataBack = 0;

    KTH71_SPI_TransmitReceive(dataSend, &dataBack, 3, 1);
    return dataBack;
}

/**
  * @brief  Unlocks registers.
  * @retval None
  */
void KTH71_UnlockReg(void)
{
    unsigned char dataSend[4] = {0x20, 0x24, 0x01, 0x01};
	unsigned char i;

	CSN_CLR;

	for(i=0;i<4;i++)
	{
        spi_3wire_send_byte(dataSend[i]);	
	}	

	CSN_SET;
}

/**
  * @brief  Locks registers.
  * @retval None
  */
void KTH71_LockReg(void)
{
    unsigned char dataSend[4] = {0x20, 0x24, 0x12, 0x31};
	unsigned char i;

	CSN_CLR;

	for(i=0;i<4;i++)
	{
		spi_3wire_send_byte(dataSend[i]);
	}

	CSN_SET;
}

/**
  * @brief  Writes the register value to MTP..
  * @retval None
  */
void KTH71_WriteRegToMTP(void)
{
    unsigned char dataSend[3] = {0x22, 0x55, 0xAA};
	unsigned char i;

	CSN_CLR;

	for(i=0;i<3;i++)
	{
		spi_3wire_send_byte(dataSend[i]);
	}

	CSN_SET;

	wk_delay_ms(400);
}

/**
  * @brief  Checks the buffer by CRC8.
  * @param  Data buffer.
  * @param  Data buffer length.
  * @retval Return 1 for success and 0 for failure
  */
static unsigned char KTH71_IsCRCOK(unsigned char *pbuf, unsigned char buflen)
{
  unsigned char crc = 0x00;
  unsigned char i;
  for (i = 0; i < buflen - 1; i++)
  {
    crc = CRC8Table[crc ^ pbuf[i]];
  }
  crc = crc ^ 0x55;
  
  return (crc == pbuf[buflen - 1]) ? 1 : 0;
}
// 接收一个字节，等待发送
unsigned char spi_3wire_receive_byte(void)
{
#if SPI_SOFT_CTRL
    unsigned char data = 0;
    
    // 设置为输入模式
    SET_MISO_INPUT;
    
    // 接收8位数据，最高位在前
    for(unsigned char i = 0; i < 8; i++)
    {
        data <<= 1;  // 移位，准备接收下一位
        
        // 第一个数据接收，CPHA=1，在第一个数据接收之后发送
        SCK_CLR;  // 发送数据，最高位在前
        
        // 等待
        delay_us(1);								//
        
        // 接收数据，最高位在前
        if(READ_MISO)
            data |= 0x01;
        
        // 第二个数据接收，CPHA=1，在第二个数据接收之后发送
        SCK_SET;  // 发送数据，最高位在前
        
        // 等待
        delay_us(1);						//
    }
    
    return data;
#else
    unsigned char data = 0;

    // 设置为输入模式
    spi_half_duplex_direction_set(SPI2, SPI_HALF_DUPLEX_DIRECTION_RX);
    
    while(spi_i2s_flag_get(SPI2, SPI_I2S_RDBF_FLAG) == RESET);
    data = spi_i2s_data_receive(SPI2);

    return data;
#endif
    
}
/**
  * @brief  Transmit and Receive data.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer.
  * @param  pRxData pointer to reception data buffer.
  * @param  Transmission data buffer length.
  * @param  Reception data buffer length.
  * @retval None
  */
static void KTH71_SPI_TransmitReceive(unsigned char *pTxData, unsigned char *pRxData, unsigned char txLen, unsigned char rxLen)
{
	unsigned char i;  
    
	CSN_CLR;

	for(i=0;i<txLen;i++)
	{	 
        
         spi_3wire_send_byte(*pTxData);
         pTxData++;
	}

	for(i=0;i<rxLen;i++)
	{

        *pRxData	=spi_3wire_receive_byte();
        pRxData++;
	}  
	CSN_SET;
}

// 第一次使用KTH7111芯片需执行以下代码
static void SetKTH71(void)
{
    unsigned char Test_temp[3];
	KTH71_ReadReg(0x11,&Test_temp[2]);
	wk_delay_ms(10);
	if(Test_temp[2]==0xac)
	{
        KTH71_UnlockReg();
        wk_delay_ms(10);
        KTH71_WriteReg(0x11,0x00);
        wk_delay_ms(10);
        KTH71_WriteRegToMTP();
        wk_delay_ms(10);

	}
	
}

void KTH71_Init(void)
{
#if SPI_SOFT_CTRL
    gpio_init_type gpio_init_struct;

    gpio_default_para_init(&gpio_init_struct);
	
	gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;					//SCK
	gpio_init_struct.gpio_pins = GPIO_PINS_8;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOB, &gpio_init_struct);
	gpio_bits_set(GPIOB, GPIO_PINS_8);

	gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;					//CAL
	gpio_init_struct.gpio_pins = GPIO_PINS_4;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOB, &gpio_init_struct);
	gpio_bits_reset(GPIOB, GPIO_PINS_4);

  /* configure the MOSI pin */

		
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;					//MISO
    gpio_init_struct.gpio_pins = GPIO_PINS_5;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_bits_set(GPIOB, GPIO_PINS_5);
		

  /* configure the CS pin */
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;					//CS
    gpio_init_struct.gpio_pins = GPIO_PINS_6;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_bits_set(GPIOB, GPIO_PINS_6);
#else
    wk_spi2_init();
#endif
    SetKTH71();
}
