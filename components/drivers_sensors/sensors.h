#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"


/* Inicializa tarefas/sensores */
esp_err_t sensors_init(void);

/* Leituras do LDR */
int sensors_get_ldr_raw(void);
int sensors_get_ldr_threshold(void);

/* 6B: delta ajustável em runtime (Blynk slider) */
int  sensors_get_ldr_delta(void);
void sensors_set_ldr_delta(int delta);

/* Estado do LED do LDR (para dashboard) */
bool sensors_is_ldr_led_on(void);
