#ifndef INC_HTS221_DRIVER_H_
#define INC_HTS221_DRIVER_H_

#include "main.h"
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;

bool hts221_driver_check(void);
bool hts221_driver_init(void);
bool hts221_driver_get_temperature(float *temperature, float *humidity);

#endif /* INC_HTS221_DRIVER_H_ */
