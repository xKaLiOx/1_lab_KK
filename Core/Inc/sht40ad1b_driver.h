#ifndef INC_SHT40AD1B_DRIVER_H_
#define INC_SHT40AD1B_DRIVER_H_

#include "main.h"
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;

bool sht40ad1b_driver_check(void);
bool sht40ad1b_driver_init(void);
bool sht40ad1b_driver_get_temperature(float *temperature, float *humidity);

#endif /* INC_SHT40AD1B_DRIVER_H_ */
