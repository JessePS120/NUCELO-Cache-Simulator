/* USER CODE BEGIN Header */
/**
 *Name: Jesse Seidel
 *CWID: 12342867
 *Lab Number: 5
 *Due Date: 4/24/2025
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//UART_Buffer for DMA.
uint8_t UART_Buffer[12] = {0};
//Largest possible cache.
uint32_t cache[2048] = {0};
//Used to keep track of indexes that are valid. Otherwise, address 0 returns a hit on the first try.
_Bool valid[2048] = {0};
//Used to keep track of what address is being sent.
uint32_t sequenceCounter = 0;
//Variables describing cache setup.
uint32_t initialAddressNum = 0;
uint32_t mainMemAmount = 0;
uint32_t cacheMemAmount = 0;
uint32_t cacheBlockSize = 0;
uint32_t wordSize = 0;
//Variables used to store information about the addresses. Calculated from cache setup variables.
uint32_t addressNum = 0;
uint32_t bitNum = 0;
uint32_t offset = 0;
uint32_t indexInfo = 0;
//Variables used to print information to UART. Note: information is printed all at once.
char stringBuffer[4096] = {'\0'};
char* bufferLoc = stringBuffer;
//Variable used to keep track of the number of cache hits.
uint32_t hits = 0;

void getCacheEntry(uint32_t address){
	uint32_t addressOffset = address % cacheBlockSize;
	uint32_t addressIndex = (address/cacheBlockSize) % (cacheMemAmount / cacheBlockSize);
	uint32_t addressTag = address / cacheMemAmount;
	//Add address information to buffer for printing.
	bufferLoc += sprintf(bufferLoc, "0x%08" PRIX32 " - 0x%08" PRIX32 " - 0x%08" PRIX32 " - 0x%08" PRIX32 " - ", address, addressTag, addressIndex, addressOffset);
	//Cache hit.
	if(valid[addressIndex] && cache[addressIndex] == addressTag){
		 bufferLoc += sprintf(bufferLoc, "hit\r\n");
		 hits++;
	}
	//Cache miss.
	else{
		//update cache to store value.
		cache[addressIndex] = addressTag;
		//Update valid array.
		valid[addressIndex] = 1;
		bufferLoc += sprintf(bufferLoc, "miss\r\n");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//Copy contents of UART_Buffer to safer buffer that is null terminated.
	uint32_t received_value = 0;
	char hex_str[13];
	memcpy(hex_str, UART_Buffer, 12);
	hex_str[12] = '\0';
	if(sscanf(hex_str, "0x%" SCNx32, &received_value) == 1){
		//HAL_UART_Transmit(&huart2, "Entering Switch\r\n", sizeof("Entering Switch\r\n"), HAL_MAX_DELAY);
		switch(sequenceCounter){
			case 0:
				//First address contains the number of addresses.
				addressNum = received_value;
				initialAddressNum = received_value;
				break;
			case 1:
				//Second address contains the size of main memory.
				mainMemAmount = received_value;
				break;
			case 2:
				//Third address contains the cache memory size.
				cacheMemAmount = received_value;
				break;
			case 3:
				//Fourth address contains the cache block size.
				cacheBlockSize = received_value;
				break;
			case 4:
				//Fifth address contains teh word size.
				wordSize = received_value;
				//Calculate cache statistics.
				bitNum = (uint8_t)log2(mainMemAmount);
				offset = (uint8_t)log2(cacheBlockSize / wordSize);
				indexInfo = (uint8_t)log2(cacheMemAmount / cacheBlockSize);
				break;
			default:
				//After the fifth address, use the sent address to access cache.
				getCacheEntry(received_value);
				addressNum--;
				//If all addresses were sent, print cache statistics over UART.
				if(addressNum == 0){
					bufferLoc += sprintf(bufferLoc, "Memory address lines = %" PRIu32 "\r\n", bitNum);
					bufferLoc += sprintf(bufferLoc, "Tag bits = %" PRIu32 "\r\n", bitNum - offset - indexInfo);
					bufferLoc += sprintf(bufferLoc, "Index bits = %" PRIu32 "\r\n", indexInfo);
					bufferLoc += sprintf(bufferLoc, "Offset bits = %" PRIu32 "\r\n", offset);
					bufferLoc += sprintf(bufferLoc, "Hit rate = %" PRIu32 "/%" PRIu32 "= %f%%\r\n", hits, initialAddressNum, (double)hits/initialAddressNum * 100);
					HAL_UART_Transmit(&huart2, (uint8_t*)stringBuffer, bufferLoc-stringBuffer, HAL_MAX_DELAY);
				}
				break;
		}
	}
	sequenceCounter++;
	//Tell DMA to get next value
	HAL_UART_Receive_DMA(&huart2, UART_Buffer, 12);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //Initiate DMA transfer 12 bytes at a time.
  HAL_UART_Receive_DMA(&huart2, UART_Buffer, 12);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
