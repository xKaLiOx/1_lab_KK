#include "gnss_driver.h"
#include <stdio.h>
#include <string.h>

/* Turn on GNSS module */
bool gnss_driver_init(void){
	HAL_GPIO_WritePin(GPS_WakeUp_GPIO_Port, GPS_WakeUp_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPS_Reset_GPIO_Port, GPS_Reset_Pin, GPIO_PIN_SET);
	return true;
}
