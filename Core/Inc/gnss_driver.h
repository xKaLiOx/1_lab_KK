#ifndef INC_GNSS_DRIVER_H_
#define INC_GNSS_DRIVER_H_

#include <stdbool.h>
#include "main.h"

typedef struct sCoordinates_t{
	float latitude_dec; /* dec - coordinates expressed in decimal format */
	float longitude_dec;
}sCoordinates_t;

bool gnss_driver_init(void);
bool gnss_Driver_parse(uint8_t *message, size_t message_length, sCoordinates_t *coordinates_out);



#endif /* INC_GNSS_DRIVER_H_ */
