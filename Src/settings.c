#include "settings.h"
#include "stm32f1xx_hal.h"
#include "main.h"

#define I2C_EEPROM_ADDRESS			0x50
#define	I2C_EEPROM_TIMEOUT			10
#define	I2C_EEPROM_WRITE_DELAY		10
#define	MEMORY_ADDRESS				0x00

#ifndef	USE_DEBUG_PRINTF
	#define		printf(...)		(0)
#endif

#include "i2c.h"

//extern I2C_HandleTypeDef hi2c1;

//static void MX_I2C1_Init (void);

int i2c_eeprom_write (uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
  uint8_t Data[I2C_EEPROM_PAGE_SIZE];
  uint16_t i = 0;
	HAL_StatusTypeDef Status;

  while (i < Size)
  {
		uint16_t j = 0;
		while ((j < I2C_EEPROM_PAGE_SIZE) && (i < Size))
		{
			Data[j] = pData[i];
			i++;
			j++;
		}
		//Status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, MemAddress, I2C_MEMADD_SIZE_16BIT, Data, j, I2C_EEPROM_TIMEOUT);
		if ( HAL_OK != HAL_I2C_Mem_Write(&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, MemAddress, I2C_MEMADD_SIZE_16BIT, Data, j, I2C_EEPROM_TIMEOUT) )
		{
			printf ("Can't write settings to EEPROM! %d \n\r", Status);
			return -1;
		}
		MemAddress += I2C_EEPROM_PAGE_SIZE;
		HAL_Delay (I2C_EEPROM_WRITE_DELAY);
  }
	return 1;
}

int i2c_eeprom_write_page (uint16_t PageAddress, uint8_t *Data)
{
	for (int i = 0; i < I2C_EEPROM_PAGE_SIZE; i++)
	{
		if ( HAL_OK != HAL_I2C_Mem_Write(&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, PageAddress, I2C_MEMADD_SIZE_16BIT, Data, I2C_EEPROM_PAGE_SIZE, I2C_EEPROM_TIMEOUT) )
		{
			//printf ("Can't write data to EEPROM! \n\r");
			return -1;
		}
	}
	return PageAddress;
}

int i2c_eeprom_read (uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
  uint8_t Data[I2C_EEPROM_PAGE_SIZE];
  uint16_t i = 0;

  while(i < Size)
  {
		uint16_t j = I2C_EEPROM_PAGE_SIZE;
		if ( HAL_OK != HAL_I2C_Mem_Read (&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, MemAddress, I2C_MEMADD_SIZE_16BIT, Data, j, I2C_EEPROM_TIMEOUT))
		{
			printf ("Can't read settings from EEPROM! \n\r");
			return -1;
		}
		j = 0;
		while((j < I2C_EEPROM_PAGE_SIZE) && (i < Size))
		{
			pData[i] = Data[j];
			i++;
			j++;
		}
		MemAddress += I2C_EEPROM_PAGE_SIZE;
		HAL_Delay(I2C_EEPROM_WRITE_DELAY);
  }
	return 1;
}

