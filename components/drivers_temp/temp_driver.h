#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t temp_driver_init(int dht_gpio, int fan_gpio, float setpoint_c);

void  temp_driver_set_point(float setpoint_c);
float temp_driver_get_point(void);
bool  temp_driver_is_fan_on(void);

float temp_driver_get_setpoint_c(void);
void  temp_driver_set_setpoint_c(float setpoint_c);

bool temp_driver_get_last_ok(float *temp_c, float *hum_percent, uint32_t *age_ms);


