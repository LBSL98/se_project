#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void  temp_driver_init(void);

float temp_driver_get_last_temperature(void);
float temp_driver_get_last_humidity(void);
bool  temp_driver_is_fan_on(void);

/* Setpoint (o que sua main quer usar) */
float temp_driver_get_point(void);
void  temp_driver_set_point(float celsius);

#ifdef __cplusplus
}
#endif
