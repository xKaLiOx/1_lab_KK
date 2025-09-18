/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* Type to describe register values */
typedef struct sRegister_t {
	uint8_t address;
	uint8_t value;
}sRegister_t;

typedef struct
{
    char Latitude[20];
    char Longitude[20];
    char Lat_direction[2];
    char Long_direction[2];

    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t Satellites;
    uint8_t fix;
    float HDOP;// precision, 1-2 excellent, >5 bad
} GPSDef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPS_WakeUp_Pin GPIO_PIN_5
#define GPS_WakeUp_GPIO_Port GPIOA
#define PPS_INT_Pin GPIO_PIN_10
#define PPS_INT_GPIO_Port GPIOB
#define PPS_INT_EXTI_IRQn EXTI15_10_IRQn
#define GPS_Reset_Pin GPIO_PIN_8
#define GPS_Reset_GPIO_Port GPIOA
#define INTERVAL_SIGNAL_Pin GPIO_PIN_10
#define INTERVAL_SIGNAL_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
//#define SCAN_I2C
//#define LOG_DATA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
