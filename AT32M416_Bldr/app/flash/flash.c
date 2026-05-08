#include "flash.h"
#include "at32m412_416_flash.h"

typedef struct _UPGRADE_RUNTIME_HEADER
{
	unsigned char	Signature[4];
	unsigned short	ImageType;
	unsigned short	Compress;
	unsigned long	ContentOffset;
	unsigned long	FileLen;
	unsigned long	Reserved0;
	unsigned long	Checksum32;	
	unsigned long	Reserved1[2];
} UPGRADE_RT_HEAD;

#define APPLICATION_ADDRESS 0x08001000
#define FIRMWARE_UPDATE_BASE_ADDR 0x08010400
#define FIRMWARE_UPDATE_MAX_PAGE_NUMBER	61
#define H2NL(x) ((x>>24 & 0x000000FF) | (x>>8 & 0x0000FF00) | (x<<24 & 0xFF000000) | (x<<8 & 0x00FF0000))
#define SECTOR_SIZE         1024

static UPGRADE_RT_HEAD firmwareHeader;

static unsigned int FirmwareChecksum32(unsigned int length)
{
	unsigned int	i;
	unsigned int	count = (length >> 2);
	unsigned int	cksum = 0, data32;
	unsigned int	firmwareAddr = FIRMWARE_UPDATE_BASE_ADDR + sizeof(UPGRADE_RT_HEAD);

	for (i = 0; i < count; i++)
	{
		data32 = *(UINT32*)firmwareAddr;
		
		cksum += data32;
		firmwareAddr += 4;
		if (cksum < data32)
		{
			cksum++;
		}
	}
	
	if (length % 4)
	{
    data32 = *(UINT32*)firmwareAddr;
		count = length % 4;

		if (count == 3)
		{
			data32 = (data32&0x00FFFFFF);
		}
		else if (count == 2)
		{
			data32 = (data32&0x0000FFFF);
		}
		else 
		{
			data32 = (data32&0x000000FF);
		}

		cksum += data32;
		if (cksum < data32)
		{
			cksum++;
		}
	}
	
	return cksum;

}

int AT32M416Flash_Erase(unsigned int sectorAddr, unsigned char numSectors)
{
    unsigned char i = 0;
   
    if(numSectors > FIRMWARE_UPDATE_MAX_PAGE_NUMBER)
        return 1;
    for(i=0;i<numSectors;i++)
    {
        if(flash_sector_erase(sectorAddr) != FLASH_OPERATE_DONE)
            return 1;
        sectorAddr += SECTOR_SIZE;
    }
    
    return 0;
}

int AT32M416Flash_Read(unsigned int startAddress, unsigned int endAddress, unsigned int *data)
{
    unsigned int u32Addr;

    if (!data)
        return 1;

    for (u32Addr = startAddress; u32Addr < endAddress; data++)
    {
        data[0] = *((UINT32 *)u32Addr);
        u32Addr += 4;
    }

    return 0;
}

void AT32M416Flash_UpdateApplication(void)
{
    uint32_t	TotalLen;
    uint32_t	StartAddress, FirmwareAdress;
    uint32_t	counter;
    uint8_t     data32[4];

    // Clean Flag
    flash_unlock();
    FLASH->sts = FLASH_OBF_FLAG | FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG | FLASH_USDERR_FLAG;
    flash_lock();

    // Read upgrade state
    StartAddress = FIRMWARE_UPDATE_BASE_ADDR;
    flash_unlock();
    AT32M416Flash_Read(StartAddress, StartAddress+32, (unsigned int *)&firmwareHeader);
    flash_lock();
    if ((firmwareHeader.Signature[0] != 'A') | (firmwareHeader.Signature[1] != 'S') |
        (firmwareHeader.Signature[2] != 'I') | (firmwareHeader.Signature[3] != 'X'))
    {
        return;
    }
    
    /* Check the file size */
    firmwareHeader.FileLen = H2NL(firmwareHeader.FileLen);
    if (FirmwareChecksum32(firmwareHeader.FileLen-sizeof(UPGRADE_RT_HEAD)) != ~firmwareHeader.Checksum32)
    {
        flash_unlock();
        AT32M416Flash_Erase(FIRMWARE_UPDATE_BASE_ADDR, 1);
        flash_lock();
        return;
    }
    
    
    StartAddress = APPLICATION_ADDRESS;
    FirmwareAdress = FIRMWARE_UPDATE_BASE_ADDR + sizeof(UPGRADE_RT_HEAD);
    /* calculate the number of sector to be erased */
    counter = (firmwareHeader.FileLen - sizeof(UPGRADE_RT_HEAD) + 1023)/SECTOR_SIZE;
    
    flash_unlock();
    /* Check the firmware to be updated */
    if (firmwareHeader.ImageType == 3)
    {
        AT32M416Flash_Erase(StartAddress, counter);
    }
    else
    {
        AT32M416Flash_Erase(FIRMWARE_UPDATE_BASE_ADDR, 1);
        flash_lock();
        return;
    }
    
    /* calculate the number of write_4bytes */
    TotalLen = (firmwareHeader.FileLen - sizeof(UPGRADE_RT_HEAD) + 3) / 4;
    for (counter = 0; counter < TotalLen; counter++)
    {
        AT32M416Flash_Read(FirmwareAdress, FirmwareAdress+4, (unsigned int *)data32);
        flash_word_program(StartAddress, *(unsigned int *)data32);
        StartAddress += 4;
        FirmwareAdress += 4;
    }
    
    AT32M416Flash_Erase(FIRMWARE_UPDATE_BASE_ADDR, 1);
    flash_lock();

}

typedef  void (*pFunction)(void);
void Jump2APP(void)
{
    unsigned int JumpAddress;
    pFunction JumpToApplication;
    
    if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t *) (APPLICATION_ADDRESS + 4);
        JumpToApplication = (pFunction) JumpAddress;

        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;

        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t *) APPLICATION_ADDRESS);
        __set_CONTROL(0);
        JumpToApplication();
        while (1);
    }
}
