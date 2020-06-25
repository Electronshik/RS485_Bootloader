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
#include "RS485_Simple_Boot.c"
#include "settings.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	MAIN_APP_START,
	WAIT_FOR_UPDATE,
	UPDATE_LOAD_TO_MEM,
	FLASH_LOAD
} boot_state_t;
typedef  void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	APP_ADDRESS		0x08004000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
boot_state_t State = MAIN_APP_START;
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

  i2c_eeprom_write (0x00, (unsigned char *)&buf, sizeof(buf));
  strcpy (buf, "Fuck that shit!");
  i2c_eeprom_read (0x00, (unsigned char *)&buf, sizeof(buf));
  printf ("Eeprom: %s", buf);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  State = WAIT_FOR_UPDATE;
	while (1)
	{
		switch (State)
		{
			case MAIN_APP_START:
					ExecMainFW ();
					HAL_GPIO_WritePin (RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
					HAL_UART_Transmit(&huart3, (uint8_t *) "Not Jumped!!!\r\n", strlen ("Not Jumped!!!\r\n"), 1000);
					HAL_GPIO_WritePin (RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
				break;
			case FLASH_LOAD:
					EraseStatus = HAL_FLASH_Unlock();
					printf ("HAL Unlock: %d\r\n", EraseStatus);
					for(int page_counter = 0; (page_counter <= 32) && (EraseStatus == HAL_OK); page_counter++)
					{
						EraseStatus = HAL_FLASHEx_Erase (&FlashPage, &PageError);
						FlashPage.PageAddress = APP_ADDRESS + (1024 * page_counter);
					}
					for (int j = 0; j <= (5212UL + 1); )
					{
						j += 4;
							WordBuf =  _acRS485_Simple_Boot[j+3] << 24;
							WordBuf |= _acRS485_Simple_Boot[j+2] << 16;
							WordBuf |= _acRS485_Simple_Boot[j+1] << 8;
							WordBuf |= _acRS485_Simple_Boot[j+0];
							EraseStatus = HAL_FLASH_Program (FLASH_TYPEPROGRAM_WORD, APP_ADDRESS+j, (uint64_t) WordBuf);
							WordBuf = 0;
					}
					EraseStatus = HAL_FLASH_Lock();
					printf ("HAL Lock: %d\r\n", EraseStatus);
					HAL_Delay (300);
					State = MAIN_APP_START;
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
