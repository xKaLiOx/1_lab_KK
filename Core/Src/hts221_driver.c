#include "hts221_driver.h"

#define HTS221_whoami_register 0x0F
#define HTS221_whoami_expected 0xBC
#define HTS221_I2C_ADDRESS 0xBF
#define I2C_TIMEOUT 10

// Configuration, calibration and variables of HTS221 sensor
#define CTRL_REG1_HTS221 0x20 // Control register 1
#define CTRL_REG1_HTS221_SETTINGS 0x82 // Data output register - 7 Hz, continuous mode
#define CTRL_REG2_HTS221 0x21 // Control register 2
#define CTRL_REG2_HTS221_SETTINGS 0x80 // Continuous mode
#define DATA_REG_HTS221 0xA8 // LSB of data
#define DATA_CONFIG_HTS221 0xB0 // Configuration register beginning
#define HTS221_SETTING_AMOUNT 2

/* lut - "Look Up Table" to store configuration data */
static const sRegister_t init_register_lut[HTS221_SETTING_AMOUNT] = {
	[0] = {.address = CTRL_REG1_HTS221, .value = CTRL_REG1_HTS221_SETTINGS},
	[1] = {.address = CTRL_REG2_HTS221, .value = CTRL_REG2_HTS221_SETTINGS}
};

static uint8_t whoami_received = 0;
static uint8_t data_raw_HTS221[16] = {0};
static uint8_t Y1_hum_raw, Y2_hum_raw = 0;
static uint16_t Y1_temp_raw, Y2_temp_raw = 0;
static int16_t X1_temp_raw, X2_temp_raw, X1_hum_raw, X2_hum_raw = 0;
static int16_t humidity_raw_HTS221 = 0;
static int16_t temperature_raw_HTS221 = 0;
static float slope_temp, slope_hum, intercept_temp, intercept_hum = 0;

/* Check WHo_AM_I register */
bool hts221_driver_check(void){
	HAL_I2C_Mem_Read(&hi2c1, HTS221_I2C_ADDRESS, HTS221_whoami_register, 1, &whoami_received, 1, 10);
	return whoami_received == HTS221_whoami_expected;
}
/* Configure sensor to read temperature and pressure data */
bool hts221_driver_init(void){
	/* Write configuration register data from init_register_lut array */
	for(int i = 0; i < HTS221_SETTING_AMOUNT; i++){
		HAL_I2C_Mem_Write(&hi2c1, HTS221_I2C_ADDRESS, init_register_lut[i].address, 1, (uint8_t *)&init_register_lut[i].value, 1, 10);
	}
	/* Read temperature and pressure calibration data */
	HAL_I2C_Mem_Read(&hi2c1, HTS221_I2C_ADDRESS, DATA_CONFIG_HTS221, 1, data_raw_HTS221, 16, 10);
	X1_temp_raw = data_raw_HTS221[12] | (data_raw_HTS221[13] << 8);
	X1_hum_raw = data_raw_HTS221[6] | (data_raw_HTS221[7] << 8);
	X2_temp_raw = data_raw_HTS221[14] | (data_raw_HTS221[15] << 8);
	X2_hum_raw = data_raw_HTS221[10] | (data_raw_HTS221[11] << 8);
	Y1_temp_raw = data_raw_HTS221[2] | ((data_raw_HTS221[5] & 0b00000011) << 8);
	Y1_hum_raw = data_raw_HTS221[0];
	Y2_temp_raw = data_raw_HTS221[3] | ((data_raw_HTS221[5] & 0b00001100) << 6);
	Y2_hum_raw = data_raw_HTS221[1];
	/* Calculate approximation data */
	slope_temp = ((float)Y2_temp_raw/8.0f-(float)Y1_temp_raw/8.0f)/(((float)X2_temp_raw)-((float)X1_temp_raw));
	slope_hum = ((float)Y2_hum_raw/2.0f-(float)Y1_hum_raw/2.0f)/(((float)X2_hum_raw)-((float)X1_hum_raw));
	intercept_temp = (float)Y1_temp_raw / 8.0f -(slope_temp*(float)X1_temp_raw );
	intercept_hum = (float)Y1_hum_raw / 2.0f -(slope_hum*(float)X1_hum_raw);
	return true;
}

bool hts221_driver_get_temperature(float *temperature_out, float *humidity_out){
	if((temperature_out == NULL) || (humidity_out == NULL)){
		return false;
	}
	// Getting HTS221 sensor data
	HAL_I2C_Mem_Read(&hi2c1, HTS221_I2C_ADDRESS, DATA_REG_HTS221, 1, data_raw_HTS221, 4, 10);
	//Processing humidity and temperature data of HTS221
	humidity_raw_HTS221 = data_raw_HTS221[0] | (data_raw_HTS221[1] << 8);
	*humidity_out = ((float)humidity_raw_HTS221 * slope_hum + intercept_hum);
	temperature_raw_HTS221 = data_raw_HTS221[2] | (data_raw_HTS221[3] << 8);
	*temperature_out = ((float)temperature_raw_HTS221 * slope_temp + intercept_temp);
	return true;
}
