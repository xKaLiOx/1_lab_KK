/*
 * LPS22HB.c
 *
 *  Created on: Sep 5, 2025
 *      Author: Linas
 */

#include "LPS22HB.h"
//@brief Rebooting and testing whoami value
HAL_StatusTypeDef LPS22HB_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
	uint8_t data = 0;
	HAL_StatusTypeDef Status = HAL_OK;
	Status = HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_WHOAMI_ADDR, sizeof(uint8_t), &data, 1, LPS22HB_TIMEOUT);

	if(data == LPS22HB_WHOAMI_VAL && Status == HAL_OK)
	{
		char MSG[] = "LPS22HB recognized \r\n";
		Status = HAL_UART_Transmit(huart, (uint8_t*)MSG, strlen(MSG), LPS22HB_TIMEOUT);
		if(Status == HAL_OK)
		{
			data = 0x80;//reboot
			Status = HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, sizeof(uint8_t), &data, 1, LPS22HB_TIMEOUT);
		}
		else return HAL_ERROR;
		if(Status == HAL_OK)
		{
			data = 0x04;//INTERRUPT_DATA_READY
			Status = HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG3, sizeof(uint8_t), &data, 1, LPS22HB_TIMEOUT);
		}
		else return HAL_ERROR;
	}
	return Status;
}

//@brief Starting one-shot mode data acquisition
HAL_StatusTypeDef LPS22HB_Start_Sample(I2C_HandleTypeDef *hi2c)
{
	uint8_t data = 0x11;
	HAL_StatusTypeDef Status = HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, sizeof(uint8_t), &data, 1, LPS22HB_TIMEOUT);
	return Status;
}

HAL_StatusTypeDef LPS22HB_Convert_Data(I2C_HandleTypeDef *hi2c, LPS22HB_STRUCT_DATA* sensor_values)
{
    const uint8_t PRESSURE_BUFFER = 3;
    const uint8_t TEMP_BUFFER = 2;
	uint8_t temp_data[TEMP_BUFFER];
	uint8_t pres_data[PRESSURE_BUFFER];

	HAL_StatusTypeDef Status;

	Status = HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_PRESS_OUT_XL_ADDR, 1, pres_data, sizeof(pres_data), LPS22HB_TIMEOUT);
	if(Status == HAL_OK)
	{
	Status = HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_TEMP_OUT_L_ADDR, 1, temp_data, sizeof(temp_data), LPS22HB_TIMEOUT);
	}
	else return HAL_ERROR;

	sensor_values->Pressure = (pres_data[0] | (pres_data[1] << 8) | pres_data[2] << 16)>>12;//padalinti per 4096, 12 bitu postumis //in hPa
	sensor_values->Temperature = (temp_data[0] | (temp_data[1] << 8))/100.0;

	return HAL_OK;
}
