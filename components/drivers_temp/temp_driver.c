#include "temp_driver.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "dht.h"

static const char *TAG = "temp_driver";

static int   s_dht_gpio = -1;
static int   s_fan_gpio = -1;
static float s_setpoint_c = 25.0f;

static volatile bool  s_fan_on     = false;
static volatile bool  s_last_ok    = false;
static volatile float s_last_temp  = 0.0f;
static volatile float s_last_hum   = 0.0f;
static volatile int64_t s_last_ok_us = 0;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void fan_apply(bool on)
{
    s_fan_on = on;
    gpio_set_level(s_fan_gpio, on ? 1 : 0);
}

static void temp_task(void *pv)
{
    (void)pv;

    vTaskDelay(pdMS_TO_TICKS(2000)); // logo no começo da task, antes do while(1)

    while (1) {
        int16_t temperature = 0;
        int16_t humidity = 0;

        const esp_err_t err = dht_read_data(DHT_TYPE_DHT11, s_dht_gpio, &humidity, &temperature);
        const int64_t now_us = esp_timer_get_time();

        if (err == ESP_OK) {
            s_last_ok = true;
            s_last_ok_us = now_us;

            s_last_temp = ((float)temperature) / 10.0f;
            s_last_hum  = ((float)humidity)    / 10.0f;

            // Controle simples ON/OFF por setpoint
            const bool need_fan = (s_last_temp >= s_setpoint_c);
            fan_apply(need_fan);

            ESP_LOGI(TAG, "T=%.1fC  H=%.0f%%  Set=%.1fC  FAN=%s",
                        (double)s_last_temp, (double)s_last_hum, (double)s_setpoint_c,
                     s_fan_on ? "ON" : "OFF");
        } else {
            s_last_ok = false;
            // Mantém última leitura válida (se existir) e não mexe em s_last_ok_us
            ESP_LOGW(TAG, "Falha leitura DHT11 (err=%d). Mantendo ultima leitura valida.", (int)err);
        }

        vTaskDelay(pdMS_TO_TICKS(2500)); // logo no começo da task, antes do while(1)

    }
}

esp_err_t temp_driver_init(int dht_gpio, int fan_gpio, float setpoint_c)
{
    if (dht_gpio < 0 || fan_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    s_dht_gpio = dht_gpio;
    gpio_set_pull_mode((gpio_num_t)s_dht_gpio, GPIO_PULLUP_ONLY);

    s_fan_gpio = fan_gpio;
    s_setpoint_c = clampf(setpoint_c, 0.0f, 60.0f);

    // FAN como saída
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << (uint32_t)s_fan_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    fan_apply(false);

    ESP_LOGI(TAG, "Init DHT11 GPIO=%d, FAN GPIO=%d, setpoint=%.1fC",
             s_dht_gpio, s_fan_gpio, (double)s_setpoint_c);

    BaseType_t ok = xTaskCreate(temp_task, "temp_task", 4096, NULL, 5, NULL);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

void temp_driver_set_point(float setpoint_c)
{
    s_setpoint_c = clampf(setpoint_c, 0.0f, 60.0f);
}

float temp_driver_get_point(void)
{
    return s_setpoint_c;
}

bool temp_driver_is_fan_on(void)
{
    return s_fan_on;
}

// ---- Compatibilidade com comm_blynk.c ----

float temp_driver_get_setpoint_c(void)
{
    return temp_driver_get_point();
}

void temp_driver_set_setpoint_c(float setpoint_c)
{
    temp_driver_set_point(setpoint_c);
}

// ---- Última leitura ----

bool temp_driver_get_last_ok(float *temp_c, float *hum_percent, uint32_t *age_ms)
{
    if (temp_c)      *temp_c = s_last_temp;
    if (hum_percent) *hum_percent = s_last_hum;

    if (age_ms) {
        if (s_last_ok_us <= 0) {
            *age_ms = UINT32_MAX;
        } else {
            const int64_t now_us = esp_timer_get_time();
            const int64_t delta_us = (now_us - s_last_ok_us);
            *age_ms = (delta_us <= 0) ? 0 : (uint32_t)(delta_us / 1000);
        }
    }

    // true = a ÚLTIMA tentativa de leitura deu OK
    return s_last_ok;
}
