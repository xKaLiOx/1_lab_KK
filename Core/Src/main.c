/* USER CODE BEGIN Header */
/* By: Simonas Riauka and Aurimas Junevičius 2025-06 */
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "hts221_driver.h"
#include "sht40ad1b_driver.h"
#include "gnss_driver.h"
#include "LPS22HB.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_MESSAGE_BUFFER_MAX_LENGTH 200
#define UART_TIMEOUT 10
#define MESSAGE_BUFFER_MAX_LENGTH 200


//#define SCAN_I2C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void I2C_scanning(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool PPS_Flag = false;
static uint8_t gps_message_buffer[GPS_MESSAGE_BUFFER_MAX_LENGTH] = {0};
static uint8_t message_buffer[MESSAGE_BUFFER_MAX_LENGTH] = {0};
static int message_buffer_length = 0;
static float humidity, temperature = 0;

volatile uint8_t SAMPLE_UPDATE = 0;//for sampling interval

LPS22HB_STRUCT_DATA LPS22HB_data;
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
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
	HAL_StatusTypeDef STATUS = HAL_OK;
#ifdef SCAN_I2C
	I2C_scanning(&huart6, &hi2c1);
#endif

	/* For IKS01A2*/
	HAL_Delay(200);//wait 200ms for RC filters to charge up
	hts221_driver_init();
	gnss_driver_init();
	LPS22HB_Init(&hi2c1, &huart6);

	while(1)
	{
	LPS22HB_Start_Sample(&hi2c1);
	uint8_t reg1, reg2, status;
	HAL_I2C_Mem_Read(&hi2c1, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG1,
			I2C_MEMADD_SIZE_8BIT, &reg1, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2,
			I2C_MEMADD_SIZE_8BIT, &reg2, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1, LPS22HB_SLAVE_ADDRESS, LPS22HB_STATUS_REG,
			I2C_MEMADD_SIZE_8BIT, &status, 1, LPS22HB_TIMEOUT);

	char message_buffer[70];
	sprintf(message_buffer,"CTRL1=0x%02X, CTRL2=0x%02X, STATUS=0x%02X\r\n", reg1, reg2, status);
	uint16_t message_buffer_length = strlen(message_buffer);
	HAL_UART_Transmit(&huart6, message_buffer, message_buffer_length, 10);

	LPS22HB_Convert_Data(&hi2c1, &LPS22HB_data);
		message_buffer_length = snprintf((char *)message_buffer,MESSAGE_BUFFER_MAX_LENGTH,"LPS22HB acquired values:\r\nPressure = %1.1f hPa, Temperature = %1.1f C\r\n",LPS22HB_data.Pressure,LPS22HB_data.Temperature);
		HAL_UART_Transmit(&huart6, message_buffer, message_buffer_length, UART_TIMEOUT);
	HAL_Delay(2000);
	}

	/* Get hts221 temperature and humidity data for IKS01A2 and IKS01A3 */
	if(hts221_driver_get_temperature(&temperature, &humidity) == true){
		message_buffer_length = snprintf((char *)message_buffer, MESSAGE_BUFFER_MAX_LENGTH, "HTS221 acquired values:\r\nTemperature - %.2f C, Humidity - %.2f\r\n", temperature, humidity);
		HAL_UART_Transmit(&huart6, message_buffer, message_buffer_length, UART_TIMEOUT);
	}

	HAL_TIM_OC_Start_IT(&htim9, TIM_CHANNEL_1);//starting first and second OC channels
	HAL_TIM_OC_Start_IT(&htim9, TIM_CHANNEL_2);//starting first and second OC channels


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		while(hi2c1.State != HAL_I2C_STATE_READY);
		LPS22HB_Start_Sample(&hi2c1);
		while(hi2c1.State != HAL_I2C_STATE_READY);
		STATUS = LPS22HB_Convert_Data(&hi2c1, &LPS22HB_data);
		if(STATUS == HAL_OK)
		{
			message_buffer_length = snprintf((char *)message_buffer,MESSAGE_BUFFER_MAX_LENGTH,"LPS22HB acquired values:\r\n"
					"Pressure = %1.1f hPa, Temperature = %1.1f C\r\n",LPS22HB_data.Pressure,LPS22HB_data.Temperature);
			HAL_UART_Transmit(&huart6, message_buffer, message_buffer_length, UART_TIMEOUT);
			HAL_Delay(200);
		}

		//		if(SAMPLE_UPDATE == 1)
		//		{
		//			while(hi2c1.State != HAL_I2C_STATE_READY);//POLLING UNTIL I2C FINISHES
		//
		//			HAL_UART_Transmit(&huart6,gps_message_buffer,GPS_MESSAGE_BUFFER_MAX_LENGTH,UART_TIMEOUT);
		//			SAMPLE_UPDATE = 0;
		//		}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8400-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 10000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 3333-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 6666-1;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPS_WakeUp_Pin|GPS_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INTERVAL_SIGNAL_GPIO_Port, INTERVAL_SIGNAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPS_WakeUp_Pin GPS_Reset_Pin */
  GPIO_InitStruct.Pin = GPS_WakeUp_Pin|GPS_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PPS_INT_Pin */
  GPIO_InitStruct.Pin = PPS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PPS_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INTERVAL_SIGNAL_Pin */
  GPIO_InitStruct.Pin = INTERVAL_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INTERVAL_SIGNAL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void I2C_scanning(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c)
{
	uint8_t Buffer[25] = {0};
	uint8_t Space[] = " - ";
	uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
	uint8_t EndMSG[] = "\r\nDone! \r\n";
	uint8_t Return;
	HAL_UART_Transmit(huart, StartMSG, strlen((char*)StartMSG), 10000);
	for(uint8_t i = 0; i < 255; i++)
	{
		Return = HAL_I2C_IsDeviceReady(hi2c, i, 2, 10000);
		if(Return != HAL_OK)
		{
			HAL_UART_Transmit(huart, Space, strlen((char*)Space), 10000);
		}
		else if(Return == HAL_OK)
		{
			sprintf((char *)Buffer,"0x%X ",i);
			HAL_UART_Transmit(huart, Buffer, strlen((char*)Buffer), 10000);
		}
	}
	HAL_UART_Transmit(huart, EndMSG, strlen((char*)EndMSG), 10000);
}
/* External GPIO interrupt handler */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/* PPS interrupt handler */
	if(GPIO_Pin == PPS_INT_Pin){
		/* PPS interrupt found, receive DMA data */
		__disable_irq();
		PPS_Flag = true;
		HAL_UART_Receive_DMA(&huart1, gps_message_buffer, GPS_MESSAGE_BUFFER_MAX_LENGTH);
		SAMPLE_UPDATE = 1;
		TIM9->CNT = 0;
		__enable_irq();
		HAL_GPIO_TogglePin(INTERVAL_SIGNAL_GPIO_Port, INTERVAL_SIGNAL_Pin);// commutation to indicate
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM9)//sample data (333.3ms interval)
	{
		__disable_irq();
		SAMPLE_UPDATE = 1;
		__enable_irq();
		HAL_GPIO_TogglePin(INTERVAL_SIGNAL_GPIO_Port, INTERVAL_SIGNAL_Pin);// commutation to indicate
	}
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
	while(1){

	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
