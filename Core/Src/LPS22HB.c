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
	Status = HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_WHOAMI_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);

	if(data == LPS22HB_WHOAMI_VAL && Status == HAL_OK)
	{
		char MSG[] = "LPS22HB recognized \r\n";
		Status = HAL_UART_Transmit(huart, (uint8_t*)MSG, strlen(MSG), LPS22HB_TIMEOUT);
		data = 0x80;//reboot

		Status = HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
		//wait until full reboot
		HAL_Delay(5);
		uint8_t reg1, reg2, statusreg;
		HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG1,
				I2C_MEMADD_SIZE_8BIT, &reg1, 1, LPS22HB_TIMEOUT);
		HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2,
				I2C_MEMADD_SIZE_8BIT, &reg2, 1, LPS22HB_TIMEOUT);
		HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_STATUS_REG,
				I2C_MEMADD_SIZE_8BIT, &statusreg, 1, LPS22HB_TIMEOUT);

		char message_buffer[70];
		sprintf(message_buffer,"CTRL1=0x%02X, CTRL2=0x%02X, STATUS=0x%02X\r\n", reg1, reg2, statusreg);
		uint16_t message_buffer_length = strlen(message_buffer);
		HAL_UART_Transmit(huart, message_buffer, message_buffer_length, 10);
		uint8_t status_int_source;
		uint16_t timeout_counter=0;
        do {
            HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_INT_SOURCE, I2C_MEMADD_SIZE_8BIT, &status_int_source, 1, LPS22HB_TIMEOUT);
            HAL_Delay(1);
            timeout_counter++;
            if (timeout_counter > 1000) { // Add a timeout to prevent infinite loop
                char err_msg[] = "LPS22HB Reboot Timeout!\r\n";
                HAL_UART_Transmit(huart, (uint8_t*)err_msg, strlen(err_msg), LPS22HB_TIMEOUT);
                return HAL_ERROR; // Or handle error appropriately
            }
        } while ((status_int_source & 0x80) != 0x00); // Wait until BOOT (bit 7) is 0
        data = 0x00; // Clear Reg2
        Status = HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	}
	return Status;
}

//@brief Starting one-shot mode data acquisition
HAL_StatusTypeDef LPS22HB_Start_Sample(I2C_HandleTypeDef *hi2c)
{
	uint8_t data = 0x01;//auto increment and one shot
	//HAL_StatusTypeDef HAL_STATUS = HAL_OK;
	HAL_I2C_Mem_Write(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &data, 1, LPS22HB_TIMEOUT);
	HAL_Delay(1);
	return HAL_OK;
}

HAL_StatusTypeDef LPS22HB_Convert_Data(I2C_HandleTypeDef *hi2c, LPS22HB_STRUCT_DATA* sensor_values)
{
	const uint8_t PRESSURE_BUFFER = 3;
	const uint8_t TEMP_BUFFER = 2;
	uint8_t temp_data[TEMP_BUFFER];
	uint8_t pres_data[PRESSURE_BUFFER];
	uint8_t status;

    do {
        HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS,
                         LPS22HB_STATUS_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         &status, 1, LPS22HB_TIMEOUT);
        HAL_Delay(1);
    } while ((status & 0x03) != 0x03); // wait for P_DA & T_DA

	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, (LPS22HB_PRESS_OUT_XL_ADDR), I2C_MEMADD_SIZE_8BIT, pres_data, 3, LPS22HB_TIMEOUT);
	int32_t raw_press = (pres_data[2] << 16) | (pres_data[1] << 8) | pres_data[0];
	if (raw_press & 0x00800000) raw_press |= 0xFF000000; // sign-extend 24-bit
	sensor_values->Pressure = raw_press / 4096.0f;
	HAL_I2C_Mem_Read(hi2c, LPS22HB_SLAVE_ADDRESS, LPS22HB_TEMP_OUT_L_ADDR, I2C_MEMADD_SIZE_8BIT, temp_data, 2, LPS22HB_TIMEOUT);
	int16_t raw_temp = (int16_t)((temp_data[1] << 8) | temp_data[0]);
	sensor_values->Temperature = raw_temp / 100.0f;

	return HAL_OK;
}
