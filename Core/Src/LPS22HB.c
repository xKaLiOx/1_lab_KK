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
	uint8_t int_source;

	HAL_StatusTypeDef Status = HAL_OK;
	Status = HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_WHOAMI_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);

	if(data != LPS22HB_WHOAMI_VAL || Status != HAL_OK)
	{
		return HAL_ERROR;
	}
	char MSG[] = "\r\nLPS22HB recognized \r\n";
	Status = HAL_UART_Transmit(huart, (uint8_t*)MSG, strlen(MSG), LPS22HB_TIMEOUT);
//	data = 0x80;//reboot

//	Status = HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
//	uint32_t tickstart = HAL_GetTick();
//	do {
//		HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &int_source, 1, LPS22HB_TIMEOUT);
//		if ((HAL_GetTick() - tickstart) > 150) return HAL_TIMEOUT;
//	} while (int_source & 0x80); // wait until BOOT = 0

	data = 0x04;
	HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	uint32_t tickstart = HAL_GetTick();
	do {
		HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS,
				LPS22HB_CTRL_REG2,
				I2C_MEMADD_SIZE_8BIT,
				&data, 1, LPS22HB_TIMEOUT);
		if(HAL_GetTick()-tickstart > 100)
		{
			sprintf(MSG,"TIMED OUT IN INIT soft reset\r\n");
			Status = HAL_UART_Transmit(huart, (uint8_t*)MSG, strlen(MSG), LPS22HB_TIMEOUT);
			return HAL_TIMEOUT;
		}
	} while ((data & 0x04) == 0x04); // wait for software reset


	data = 0x04;
	HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	data = 0x00;
	HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG3, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);

	HAL_Delay(20);

	//force a measurement to clear old data
	LPS22HB_Start_Sample(hi2c,huart);
	tickstart = HAL_GetTick();
	do {
		HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS,
				LPS22HB_STATUS_REG,
				I2C_MEMADD_SIZE_8BIT,
				&data, 1, LPS22HB_TIMEOUT);
		if(HAL_GetTick()-tickstart > 100)
		{
			sprintf(MSG,"TIMED OUT IN INIT\r\n");
			Status = HAL_UART_Transmit(huart, (uint8_t*)MSG, strlen(MSG), LPS22HB_TIMEOUT);
			return HAL_TIMEOUT;
		}
	} while ((data & 0x03) <= 0x00); // wait for P_DA & T_DA

	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, (LPS22HB_PRESS_OUT_XL_ADDR), I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, (LPS22HB_PRESS_OUT_XL_ADDR+1), I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, (LPS22HB_PRESS_OUT_XL_ADDR+2), I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_TEMP_OUT_L_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, (LPS22HB_TEMP_OUT_L_ADDR+1), I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);

	return HAL_OK;
}

//@brief Starting one-shot mode data acquisition
HAL_StatusTypeDef LPS22HB_Start_Sample(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
	char message_buffer[30];
	uint8_t data;
	uint32_t tickstart = HAL_GetTick();
	do {
		HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
		if(HAL_GetTick()-tickstart > 100)
		{
			sprintf(message_buffer,"TIMED OUT IN SAMPLING START\r\n");
			HAL_UART_Transmit(huart, (uint8_t*)message_buffer, strlen(message_buffer), LPS22HB_TIMEOUT);
			return HAL_TIMEOUT;
		}
	} while (data & 0x01); // Wait until ONE_SHOT bit is cleared and if locked on reboot
	data = 0x01;//one shot mode
	LPS22HB_Log_Data(hi2c, huart);
	HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	return HAL_OK;
}

HAL_StatusTypeDef LPS22HB_Convert_Data(I2C_HandleTypeDef *hi2c, LPS22HB_STRUCT_DATA* sensor_values)
{
	const uint8_t PRESSURE_BUFFER = 3;
	const uint8_t TEMP_BUFFER = 2;
	uint8_t temp_data[TEMP_BUFFER];
	uint8_t pres_data[PRESSURE_BUFFER];
	uint8_t status;
	uint32_t tickstart = HAL_GetTick();
	do {
		HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS,
				LPS22HB_STATUS_REG,
				I2C_MEMADD_SIZE_8BIT,
				&status, 1, LPS22HB_TIMEOUT);
		if(HAL_GetTick()-tickstart > 100) return HAL_TIMEOUT;
	} while ((status & 0x03) < 0x00); // wait for P_DA & T_DA
	HAL_Delay(1);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, (LPS22HB_PRESS_OUT_XL_ADDR), I2C_MEMADD_SIZE_8BIT, &pres_data[0], 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, (LPS22HB_PRESS_OUT_XL_ADDR+1), I2C_MEMADD_SIZE_8BIT, &pres_data[1], 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, (LPS22HB_PRESS_OUT_XL_ADDR+2), I2C_MEMADD_SIZE_8BIT, &pres_data[2], 1, LPS22HB_TIMEOUT);
	int32_t raw_press = (pres_data[2] << 16) | (pres_data[1] << 8) | pres_data[0];
	if (raw_press & 0x00800000) raw_press |= 0xFF000000; // sign-extend 24-bit
	sensor_values->Pressure = raw_press / 4096.0f;
	//HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_TEMP_OUT_L_ADDR, I2C_MEMADD_SIZE_8BIT, temp_data, 2, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_TEMP_OUT_L_ADDR, I2C_MEMADD_SIZE_8BIT, &temp_data[0], 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, (LPS22HB_TEMP_OUT_L_ADDR+1), I2C_MEMADD_SIZE_8BIT, &temp_data[1], 1, LPS22HB_TIMEOUT);
	int16_t raw_temp = (int16_t)((temp_data[1] << 8) | temp_data[0]);
	sensor_values->Temperature = raw_temp / 100.0f;

	return HAL_OK;
}

HAL_StatusTypeDef LPS22HB_Log_Data(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
	uint8_t reg1, reg2, status,reg3,fifo,res_config,int_source;
	char message_buffer[120];

	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG1,
			I2C_MEMADD_SIZE_8BIT, &reg1, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2,
			I2C_MEMADD_SIZE_8BIT, &reg2, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG3,
			I2C_MEMADD_SIZE_8BIT, &reg3, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_STATUS_REG,
			I2C_MEMADD_SIZE_8BIT, &status, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_FIFO_CTRL,
			I2C_MEMADD_SIZE_8BIT, &fifo, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_RES_CONF,
			I2C_MEMADD_SIZE_8BIT, &res_config, 1, LPS22HB_TIMEOUT);
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_INT_SOURCE,
			I2C_MEMADD_SIZE_8BIT, &int_source, 1, LPS22HB_TIMEOUT);

	sprintf(message_buffer,"CTRL1=0x%02X, CTRL2=0x%02X CTRL3=0x%02X, STATUS=0x%02X, FIFO=0x%02X, RES_CONFIG=0x%02X, INT_SOURCE=0x%02X\r\n", reg1, reg2,reg3,status,fifo,res_config,int_source);
	uint16_t message_buffer_length = strlen((char *)message_buffer);
	HAL_UART_Transmit(huart, (uint8_t*)message_buffer, message_buffer_length, 10);
	return HAL_OK;
}
