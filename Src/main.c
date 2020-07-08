/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "settings.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef  void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	APP_ADDRESS				0x08004000
#define STM_FLASH_PAGE_BYTES		1024
#define LINES_PER_EEPROM_PAGE	I2C_EEPROM_PAGE_SIZE/16
#define EEPROM_START_ADDR		I2C_EEPROM_PAGE_SIZE*2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
boot_state_t State = MAIN_APP_START;
uint8_t Hex_Data_Size, Hex_Data_Type, check_sum;
uint16_t Hex_Data_Address;
uint8_t Answer_Arr[8];
uint32_t program_data;//????? ??????? ??????? ?? ????
    uint8_t Hex_Data_Calculated_Crc = 0 , Hex_Data_Income_Crc = 0;
uint8_t MemPageBuf[I2C_EEPROM_PAGE_SIZE];
int MemPageCounter = 0;
int MemPageAddress = EEPROM_START_ADDR;
int EndDataAddress = 0x08010000;
int AppSize = 0xC000;
uint8_t Flash_Buf[STM_FLASH_PAGE_BYTES];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ExecMainFW (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc (int ch, FILE *f)
{
	HAL_GPIO_WritePin (RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart3, (uint8_t *) &ch, 1, 1000);
	HAL_GPIO_WritePin (RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
	return ch;
}

void RS485_Transmit (uint8_t Tx_Buff[], uint16_t size)
{
	HAL_GPIO_WritePin (RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart3, (uint8_t *) Tx_Buff, size, 1000);
	HAL_GPIO_WritePin (RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}

void AsciiToHex (uint8_t *buf, uint8_t count)
{
	for(uint8_t i = 0; i < count; i++)
	{
		if ((buf[i] >= '0') && (buf[i] <= '9'))
		{
			buf[i] -= 0x30;
		}
		else
		{
			buf[i] -= 0x37;
		}	
	}	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	FLASH_EraseInitTypeDef FlashPage;
	HAL_StatusTypeDef EraseStatus;
	uint32_t PageError;

	FlashPage.TypeErase = FLASH_TYPEERASE_PAGES;
	FlashPage.Banks = FLASH_BANK_1;
	FlashPage.NbPages = 1;
	FlashPage.PageAddress = APP_ADDRESS;

	uint32_t WordBuf;
	char buf [32] = "Hello!\r\n";
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
// For I2c stm32f103 bug
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_Delay(100);
	__HAL_RCC_I2C1_FORCE_RESET();
	HAL_Delay(100);
	__HAL_RCC_I2C1_RELEASE_RESET();
	HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	Settings_Init (false);
  //i2c_eeprom_write (0x00, (unsigned char *)&buf, sizeof(buf));
  //strcpy (buf, "Fuck that shit!");
  //i2c_eeprom_read (0x00, (unsigned char *)&buf, sizeof(buf));
  //printf ("Eeprom: %s", buf);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	for (int i = 0; i < 32; i++) buf[i] = 0;
	RS485_Transmit ((uint8_t *) "M03:RDY", 7);
	//HAL_UART_Receive (&huart3, (uint8_t *) buf, 7, 8000);
	State = MAIN_APP_START;
	//printf ("MAIN: %s", buf);
	HAL_Delay (30);
	for (int i = 0; i < 32; i++)
	{
		HAL_UART_Receive (&huart3, (uint8_t *) buf, 1, 100);
		if (buf[0] == 'M')
		{
			HAL_UART_Receive (&huart3, (uint8_t *) buf, 1, 100);
			if (buf[0] == '0')
			{
				HAL_UART_Receive (&huart3, (uint8_t *) buf, 1, 100);
				if (buf[0] == '1')
				{
					HAL_UART_Receive (&huart3, (uint8_t *) buf, 1, 100);
					if (buf[0] == ':')
					{
						HAL_UART_Receive (&huart3, (uint8_t *) buf, 3, 100);
						if ((buf[0] == 'R') && (buf[1] == 'D') && (buf[2] == 'Y'))
						{
							HAL_Delay (30);
							RS485_Transmit ((uint8_t *) "M03:RDY", 7);
							State = WAIT_FOR_UPDATE;
							EndDataAddress = 0x0000;
							Answer_Arr [4] = ':';
							Answer_Arr [7] = 0;
							break;
						}
					}
				}
			}
		}
		HAL_Delay (30);
	}
	while (1)
	{
		switch (State)
		{
			case MAIN_APP_START:
					RS485_Transmit ((uint8_t *) "MAIN_APP_START..\r\n", 18);
					ExecMainFW ();
					RS485_Transmit ((uint8_t *) "Not Jumped!!!\r\n", 15);
				break;
			case UPDATE_LOADED_TO_MEM:
					EraseStatus = HAL_FLASH_Unlock();
					for(int page_counter = 0; (page_counter < 32) && (EraseStatus == HAL_OK); page_counter++)
					{
						EraseStatus = HAL_FLASHEx_Erase (&FlashPage, &PageError);
						FlashPage.PageAddress = APP_ADDRESS + (STM_FLASH_PAGE_BYTES * page_counter);
					}
					AppSize = ((APP_ADDRESS + EndDataAddress) - APP_ADDRESS) / STM_FLASH_PAGE_BYTES;
					printf ("AppSize: %d\r\n", AppSize);
					for (int j = 0; j <= AppSize; j++)
					{
						if (i2c_eeprom_read ((j*STM_FLASH_PAGE_BYTES + EEPROM_START_ADDR), Flash_Buf, STM_FLASH_PAGE_BYTES) > 0)
						{
							for (int k = 0; k < STM_FLASH_PAGE_BYTES; k += 4)
							{
								WordBuf =  Flash_Buf[k+3] << 24;
								WordBuf |= Flash_Buf[k+2] << 16;
								WordBuf |= Flash_Buf[k+1] << 8;
								WordBuf |= Flash_Buf[k+0];
								EraseStatus = HAL_FLASH_Program (FLASH_TYPEPROGRAM_WORD, APP_ADDRESS + k + j*STM_FLASH_PAGE_BYTES, (uint64_t) WordBuf);
								WordBuf = 0;
							}
						}
						//printf ("HAL_FLASH_Program: %d\r\n", EraseStatus);
						//printf ("j: %x \r\n", APP_ADDRESS + j*STM_FLASH_PAGE_BYTES);
					}
					EraseStatus = HAL_FLASH_Lock();
					RS485_Transmit ((uint8_t *) "HAL Flash locked\r\n", 18);
					State = MAIN_APP_START;
				break;
			case WAIT_FOR_UPDATE:
				HAL_UART_Receive (&huart3, (uint8_t *) buf, 1, 1000);
				if (buf[0] == ':' )
				{
					HAL_UART_Receive (&huart3, (uint8_t *) buf, 8, 1000);
					Answer_Arr [0] = buf[2];
					Answer_Arr [1] = buf[3];
					Answer_Arr [2] = buf[4];
					Answer_Arr [3] = buf[5];
					AsciiToHex ((uint8_t*) buf, 8);
					Hex_Data_Size = 2*(buf[1] + 16*buf[0]);
					Hex_Data_Address = buf[5] + 16*buf[4] + 256*buf[3] + 4096*buf[2];
					Hex_Data_Type = buf[7] + 16*buf[6];
					Hex_Data_Calculated_Crc = Hex_Data_Size/2 + (uint8_t)Hex_Data_Address + (uint8_t)(Hex_Data_Address>>8) + Hex_Data_Type;
					if(Hex_Data_Type == 0x00)
					{
						HAL_UART_Receive (&huart3, (uint8_t *) buf, Hex_Data_Size, 2000);
						AsciiToHex ((uint8_t*) buf, Hex_Data_Size);
						int j = 0;
						for(int i = 0; i < Hex_Data_Size; i += 2)
						{
							j = (i + 32*MemPageCounter) / 2;
							MemPageBuf[j] = (buf[i+1] + 16*buf[i]);
							Hex_Data_Calculated_Crc += MemPageBuf[j];
							EndDataAddress++;
						}
						//HAL_Delay (180);
						//printf ("MemPageBuf: %02x %02x %02x %02x %02x %02x %02x %02x \r\n", MemPageBuf[7], MemPageBuf[6], MemPageBuf[5], MemPageBuf[4], MemPageBuf[3], MemPageBuf[2], MemPageBuf[1], MemPageBuf[0]);
						HAL_UART_Receive (&huart3, (uint8_t *) buf, 2, 2000);
						AsciiToHex ((uint8_t*) buf, 2);
						Hex_Data_Income_Crc = (buf[1] + 16*buf[0]);
						Hex_Data_Calculated_Crc = 256 - Hex_Data_Calculated_Crc;
						if (Hex_Data_Calculated_Crc == Hex_Data_Income_Crc)
						{
							Answer_Arr [5] = 'O';
							Answer_Arr [6] = 'K';
							if (++MemPageCounter >= LINES_PER_EEPROM_PAGE)
							{
								MemPageCounter = 0;
								int tmp;
								tmp = i2c_eeprom_write_page (MemPageAddress, MemPageBuf);
								if (tmp > 0)
								{
									MemPageAddress = tmp;
								}
								else
								{
									MemPageCounter = LINES_PER_EEPROM_PAGE - 1;
									Answer_Arr [5] = 'E';
									Answer_Arr [6] = 'R';
									RS485_Transmit (Answer_Arr, 7);
									State = MAIN_APP_START;
									break;
								}
							}
						}
						else
						{
							Answer_Arr [5] = 'N';
							Answer_Arr [6] = 'O';
						}
						RS485_Transmit (Answer_Arr, 7);
						//printf ("Hex_Data_Calculated_Crc: %02x \r\n", Hex_Data_Calculated_Crc);
					}
					else if (Hex_Data_Type == 0x01)
					{
						State = UPDATE_LOADED_TO_MEM;
						if (MemPageCounter != 0)
						{
							Answer_Arr [5] = 'O';
							Answer_Arr [6] = 'K';
							int tmp;
								tmp = i2c_eeprom_write_page (MemPageAddress, MemPageBuf);
								if (tmp > 0)
								{
									MemPageAddress = tmp;
								}
								else
								{
									MemPageCounter = LINES_PER_EEPROM_PAGE - 1;
									Answer_Arr [5] = 'E';
									Answer_Arr [6] = 'R';
									State = MAIN_APP_START;
								}
							RS485_Transmit (Answer_Arr, 7);
						}
						//EndDataAddress = MemPageAddress + 64;
						HAL_Delay (15);
						printf ("\r\n Running main.. EndDAddr: %x; MemPageAddr: %x\r\n", EndDataAddress, MemPageAddress);
					}
					else if (Hex_Data_Type == 0x05)
					{
						//printf ("End, 0x05: \r\n");
						HAL_Delay (5);
						Answer_Arr [5] = 'O';
						Answer_Arr [6] = 'K';
						RS485_Transmit (Answer_Arr, 7);
					}
					else
					{
						HAL_Delay (5);
						Answer_Arr [5] = 'O';
						Answer_Arr [6] = 'K';
						RS485_Transmit (Answer_Arr, 7);
					}
				}
				break;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void ExecMainFW (void)
{
/*
 RCC->APB1RSTR = 0xFFFFFFFF; RCC->APB1RSTR = 0x0; 
 RCC->APB2RSTR = 0xFFFFFFFF; RCC->APB2RSTR = 0x0; 
 RCC->APB1ENR = 0x0;
 RCC->APB2ENR = 0x0;
 RCC->AHBENR = 0x0;
	HAL_UART_DeInit (&huart3);
	HAL_UART_MspDeInit (&huart3);
	HAL_GPIO_DeInit (GPIOA, GPIO_PIN_10);
	HAL_GPIO_DeInit (RS485_DIR_GPIO_Port, RS485_DIR_Pin);
	HAL_RCC_DeInit();  
	HAL_MspDeInit ();
	HAL_DeInit ();
*/
	SCB->VTOR = APP_ADDRESS;
	uint32_t jumpAddress = *((__IO uint32_t*) (APP_ADDRESS + 4)); 
	pFunction Jump_To_Application = (pFunction) jumpAddress;
	__disable_irq();
	__set_MSP(*(__IO uint32_t*) APP_ADDRESS); 
	Jump_To_Application(); 
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
