#ifndef STEP42_SENSORS_H
#define STEP42_SENSORS_H

#include <stdint.h>

#define STEP42_SENSOR_TEMP_VALID (0x01U)
#define STEP42_SENSOR_TEMP_WARNING (0x02U)
#define STEP42_SENSOR_TEMP_SHUTDOWN (0x04U)

void Step42Sensors_Init(void);
void Step42Sensors_Process(void);
float Step42Sensors_GetTemperatureC(void);
uint16_t Step42Sensors_GetAdcRaw(void);
uint8_t Step42Sensors_GetFlags(void);

#endif
