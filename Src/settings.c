#include "settings.h"
#include "stm32f1xx_hal.h"

#define I2C_EEPROM_ADDRESS			0x50
#define	I2C_EEPROM_TIMEOUT			10
#define	I2C_EEPROM_WRITE_DELAY		10
#define	SETTING_PAGE_ADDR			0x00
#define	SETTING_BACKUP_PAGE_ADDR	I2C_EEPROM_PAGE_SIZE

#ifndef	USE_DEBUG_PRINTF
	#define		printf(...)		(0)
#endif

#include "i2c.h"

Settings_t Settings;

static unsigned char writing_labels[4], writing_tag[1];

//extern I2C_HandleTypeDef hi2c1;
static void I2C_ClearBusyFlagErratum (I2C_HandleTypeDef *hi2c, uint32_t timeout);
static uint8_t wait_for_gpio_state_timeout (GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout);
//static void MX_I2C1_Init (void);
static void Settings_Structure_Init (void);
void Settings_Save (void);

void Settings_Init (bool reset_to_default)
{
	Settings_Structure_Init();
	MX_I2C1_Init();
	HAL_Delay(I2C_EEPROM_WRITE_DELAY);
#ifdef	USE_DEFAULT_SETTINGS
	reset_to_default = true;
#endif
	if (!reset_to_default)
	{
		if (i2c_eeprom_read(SETTING_PAGE_ADDR, (uint8_t *)&Settings, sizeof(Settings)) > 0)
		{
			if ((Settings.writing_tag_begin == Settings.writing_tag_end) && (Settings.writing_tag_begin != 0x00) && (Settings.writing_tag_begin != 0xFF))
			{
				printf ("Settings was successfully readed from EEPROM \n\r");
			}
			else
			{
				printf ("Data from EEPROM is incorrect! Trying to read backup..\n\r");
				if (i2c_eeprom_read(SETTING_BACKUP_PAGE_ADDR, (uint8_t *)&Settings, sizeof(Settings)) > 0)
				{
					if ((Settings.writing_tag_begin == Settings.writing_tag_end) && (Settings.writing_tag_begin != 0x00) && (Settings.writing_tag_begin != 0xFF))
					{
						printf ("Backup from EEPROM is successfully readed! Saving settings.. \n\r");
						Settings_Save();
					}
					else
					{
						printf ("Backup data from EEPROM is incorrect! Reset to default..\n\r");
						Settings_Structure_Init();
						Settings_Save();
						printf ("Settings is reseted to default! \n\r");
					}
				}
				else
				{
					Settings_Structure_Init();
					printf ("Settings is reseted to default! \n\r");
				}
			}
		}
		else
		{
			Settings_Structure_Init();
			printf ("Settings is reseted to default! \n\r");
		}
	}
	else
	{
		printf ("Settings is reseted to default! \n\r");
		Settings_Save();
	}
}

void Settings_Structure_Init (void)
{
	//Settings.Echo = _DEF_ECHO_ON;
}

void Settings_Save (void)
{
	if ( HAL_OK != HAL_I2C_Mem_Read (&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, SETTING_PAGE_ADDR, I2C_MEMADD_SIZE_16BIT, writing_tag, 1, I2C_EEPROM_TIMEOUT) )
	{
		printf ("Can't read settings from EEPROM! \n\r");
		//Global.SetErrorState (EEPROM_ERR, READ_ERROR);
	}
	Settings.writing_tag_begin = ++writing_tag[0];
	if ((Settings.writing_tag_begin == 0x00) || (Settings.writing_tag_begin == 0xFF))	Settings.writing_tag_begin = 0x01;
	Settings.writing_tag_end = Settings.writing_tag_begin;
	if (i2c_eeprom_write(SETTING_PAGE_ADDR, (uint8_t *)&Settings, sizeof(Settings)))
	{
			HAL_I2C_Mem_Read (&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, SETTING_PAGE_ADDR, I2C_MEMADD_SIZE_16BIT, writing_tag, 1, I2C_EEPROM_TIMEOUT);
			writing_labels[0] = writing_tag[0];
			HAL_I2C_Mem_Read (&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, SETTING_PAGE_ADDR + sizeof(Settings) - 1, I2C_MEMADD_SIZE_16BIT, writing_tag, 1, I2C_EEPROM_TIMEOUT);
			writing_labels[1] = writing_tag[0];
		if (writing_labels[0] == writing_labels[1])
		{
			if (i2c_eeprom_write (SETTING_BACKUP_PAGE_ADDR, (uint8_t *)&Settings, sizeof(Settings)))
			{
				HAL_I2C_Mem_Read (&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, SETTING_PAGE_ADDR, I2C_MEMADD_SIZE_16BIT, writing_tag, 1, I2C_EEPROM_TIMEOUT);
				writing_labels[0] = writing_tag[0];
				HAL_I2C_Mem_Read (&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, SETTING_PAGE_ADDR + sizeof(Settings) - 1, I2C_MEMADD_SIZE_16BIT, writing_tag, 1, I2C_EEPROM_TIMEOUT);
				writing_labels[1] = writing_tag[0];
				HAL_I2C_Mem_Read (&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, SETTING_BACKUP_PAGE_ADDR, I2C_MEMADD_SIZE_16BIT, writing_tag, 1, I2C_EEPROM_TIMEOUT);
				writing_labels[2] = writing_tag[0];
				HAL_I2C_Mem_Read (&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, SETTING_BACKUP_PAGE_ADDR + sizeof(Settings) - 1, I2C_MEMADD_SIZE_16BIT, writing_tag, 1, I2C_EEPROM_TIMEOUT);
				writing_labels[3] = writing_tag[0];
				if (writing_labels[0] == writing_labels[1] && writing_labels[1] == writing_labels[2] && writing_labels[2] == writing_labels[3])
				{
					printf ("Settings was successfully written to EEPROM \n\r");
				}
				else
				{
					printf ("Can't write settings to EEPROM! \n\r");
					//Global.SetErrorState (EEPROM_ERR, WRITE_ERROR);
					//return -1;
				}
			}
		}
		else
		{
			printf ("Can't write settings to EEPROM! \n\r");
			//Global.SetErrorState (EEPROM_ERR, WRITE_ERROR);
			//return -1;
		}
	}
}

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

int i2c_eeprom_erase (void)
{
	uint8_t Data[I2C_EEPROM_PAGE_SIZE];
	for (int i = 0; i < I2C_EEPROM_PAGE_SIZE; i++)
	{
		Data[i] = 0xFF;
	}
	for (int i = 0; i < 512; i++)
	{
		if ( HAL_OK != HAL_I2C_Mem_Write(&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, i * I2C_EEPROM_PAGE_SIZE, I2C_MEMADD_SIZE_16BIT, Data, I2C_EEPROM_PAGE_SIZE, I2C_EEPROM_TIMEOUT) )
		{
			//printf ("Can't write data to EEPROM! \n\r");
			return -1;
		}
	}
}

int i2c_eeprom_write_page (uint16_t PageAddress, uint8_t *Data)
{
	//for (int i = 0; i < I2C_EEPROM_PAGE_SIZE; i++)
	//{
		if ( HAL_OK != HAL_I2C_Mem_Write(&hi2c1, (uint16_t)I2C_EEPROM_ADDRESS << 1, PageAddress, I2C_MEMADD_SIZE_16BIT, Data, I2C_EEPROM_PAGE_SIZE, I2C_EEPROM_TIMEOUT) )
		{
			//printf ("Can't write data to EEPROM! \n\r");
			I2C_ClearBusyFlagErratum (&hi2c1, 1000);
			return -1;
		}
	//}
	return PageAddress + I2C_EEPROM_PAGE_SIZE;
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
			//printf ("Can't read settings from EEPROM! \n\r");
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

static void I2C_ClearBusyFlagErratum (I2C_HandleTypeDef *hi2c, uint32_t timeout)
{
        // 2.13.7 I2C analog filter may provide wrong value, locking BUSY. STM32F10xx8 STM32F10xxB Errata sheet

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // 1. Clear PE bit.
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(hi2c);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = GPIO_PIN_6; // SCL // ???? ??? ??????, ?? ??????? ??????
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); // ???? ???? ??????, ?? ??????? ?????? ????? GPIO?, ? ???? ??? ??? ????? ? ???? ????????? ?? ????

    GPIO_InitStructure.Pin = GPIO_PIN_7; // SDA
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_6, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_7, GPIO_PIN_SET, timeout);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET, timeout);

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET, timeout);

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_6, GPIO_PIN_SET, timeout);

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_7, GPIO_PIN_SET, timeout);

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    //GPIO_InitStructure.Alternate = GPIO_AF4_I2C2; // F4

    GPIO_InitStructure.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    __ASM volatile ("NOP");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    __ASM volatile ("NOP");

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
    __ASM volatile ("NOP");

    // Call initialization function.
    HAL_I2C_Init(hi2c);
}

static uint8_t wait_for_gpio_state_timeout (GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
 {
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 1;

    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);) // Wait until flag is set
    {
        if(timeout != HAL_MAX_DELAY) // Check for the timeout
        {
            if((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout)) ret = 0;
        }

        __ASM volatile ("NOP");
    }
    return ret;
}



