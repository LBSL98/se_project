#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sensors_init(void);

int sensors_get_ldr_raw(void);
int sensors_get_ldr_threshold(void);

#ifdef __cplusplus
}
#endif
