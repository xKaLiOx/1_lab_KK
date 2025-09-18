/*
 * LPS22HB.h
 *
 *  Created on: Sep 5, 2025
 *      Author: Linas
 */


#ifndef INC_LPS22HB_H_
#define INC_LPS22HB_H_

#include "main.h"
#include <stdio.h>

#define LPS22HB_SLAVE_ADDRESS 0xBB // SHIFTED TO LEFT BY 1

#define LPS22HB_WHOAMI_ADDR 0x0F
#define LPS22HB_WHOAMI_VAL 0b10110001//gcc compiler allows this
#define LPS22HB_PRESS_OUT_XL_ADDR 0x28
#define LPS22HB_PRESS_OUT_H_ADDR 0x2A
#define LPS22HB_TEMP_OUT_L_ADDR 0x2B
#define LPS22HB_TEMP_OUT_H_ADDR 0x2C

#define LPS22HB_CTRL_REG1 0x10
#define LPS22HB_CTRL_REG2 0x11 //7th bit reboot, 4th auto increment addr, 0th one-shot for conversion
#define LPS22HB_CTRL_REG3 0x12
#define LPS22HB_INT_SOURCE 0x25
#define LPS22HB_STATUS_REG 0x27
#define LPS22HB_FIFO_CTRL 0x14
#define LPS22HB_RES_CONF 0x1A

#define LPS22HB_TIMEOUT 10//10ms timeout delay

typedef struct
{
	float Pressure;
	float Temperature;
} LPS22HB_STRUCT_DATA;


HAL_StatusTypeDef LPS22HB_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);//reboot and interrupt config
HAL_StatusTypeDef LPS22HB_Start_Sample(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);//one-shot start
HAL_StatusTypeDef LPS22HB_Convert_Data(I2C_HandleTypeDef *hi2c, LPS22HB_STRUCT_DATA* sensor_values);//data conversion in hpa
HAL_StatusTypeDef LPS22HB_Log_Data(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

#endif /* INC_LPS22HB_H_ */
