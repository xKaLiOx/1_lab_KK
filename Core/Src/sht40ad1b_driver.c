#include "sht40ad1b_driver.h"

#define I2C_ADDRESS 0x88

#define ARRAY_LENGTH(x) (sizeof(x) / sizeof((x)[0]))

static uint8_t raw_data[6] = {0};
static uint8_t conversion_command = 0xFD;

/* Device has no WHO_AM_I register, check if address is correct */
bool sht40ad1b_driver_check(void){
	HAL_StatusTypeDef  status = HAL_I2C_IsDeviceReady(&hi2c1, I2C_ADDRESS, 3, 10);
	return status == HAL_OK;
}

/* No configuration needed */
bool sht40ad1b_driver_init(void){
	return true;
}

bool sht40ad1b_driver_get_temperature(float *temperature_out, float *humidity_out){
	if((temperature_out == NULL) || (humidity_out == NULL)){
		return false;
	}
	/* Write conversion command */

	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, &conversion_command, 1, 10);
	/* Wait for conversion */
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS, raw_data, ARRAY_LENGTH(raw_data), 10);
	uint16_t t_ticks = raw_data[0] << 8 | raw_data[1];
	uint16_t rh_ticks = raw_data[3] << 8 | raw_data[4];
	*temperature_out = -45 + 175 * (float)t_ticks/65535;
	*humidity_out = -6 + 125 * (float)rh_ticks/65535;
	if(*humidity_out > 100) {
		*humidity_out = 100;
	} else if (*humidity_out < 0) {
		*humidity_out = 0;
	}
	return true;
}
