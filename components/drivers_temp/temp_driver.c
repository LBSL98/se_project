#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "dht.h"            // esp-idf-lib/dht

#include "temp_driver.h"

/* ===== Hardware (SEU CABEAMENTO ATUAL) ===== */
#define DHT_GPIO   GPIO_NUM_17   // DHT11 DATA
#define FAN_GPIO   GPIO_NUM_16   // FAN (transistor) control

static const char *TAG = "temp_driver";

/* Controle */
static float s_setpointC = 25.0f;
static const float s_hystC = 1.0f;

/* Estado */
static bool  s_fanCmd        = false;
static float s_last_tempC    = 0.0f;
static float s_last_humidity = 0.0f;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static void temp_driver_task(void *pv)
{
    (void)pv;

    // DHT11 costuma precisar de um tempo após boot
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        float temp = 0.0f, hum = 0.0f;

        // Tenta ler (não leia mais rápido que ~1s no DHT11)
        esp_err_t err = dht_read_float_data(DHT_TYPE_DHT11, DHT_GPIO, &hum, &temp);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "FAIL lendo DHT11 (%s)", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(2500));
            continue;
        }

        portENTER_CRITICAL(&s_mux);
        s_last_tempC    = temp;
        s_last_humidity = hum;

        float sp = s_setpointC;

        // histerese
        if (!s_fanCmd && temp >= (sp + s_hystC)) {
            s_fanCmd = true;
        } else if (s_fanCmd && temp <= (sp - s_hystC)) {
            s_fanCmd = false;
        }

        bool fan = s_fanCmd;
        portEXIT_CRITICAL(&s_mux);

        gpio_set_level(FAN_GPIO, fan ? 1 : 0);

        ESP_LOGI(TAG, "T=%.1fC  H=%.0f%%  Set=%.1fC  FAN=%s",
                 temp, hum, sp, fan ? "ON" : "OFF");

        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}

void temp_driver_init(void)
{
    // FAN: saída
    gpio_config_t fan_conf = {0};
    fan_conf.pin_bit_mask = (1ULL << FAN_GPIO);
    fan_conf.mode         = GPIO_MODE_OUTPUT;
    fan_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    fan_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    fan_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&fan_conf);
    gpio_set_level(FAN_GPIO, 0);

    // DHT: pull-up interno ajuda (mas o ideal é ter pull-up externo também)
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "Init DHT11 GPIO=%d, FAN GPIO=%d, setpoint=%.1fC",
             (int)DHT_GPIO, (int)FAN_GPIO, s_setpointC);

    xTaskCreate(temp_driver_task, "temp_driver_task", 4096, NULL, 5, NULL);
}

float temp_driver_get_last_temperature(void)
{
    portENTER_CRITICAL(&s_mux);
    float v = s_last_tempC;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

float temp_driver_get_last_humidity(void)
{
    portENTER_CRITICAL(&s_mux);
    float v = s_last_humidity;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

bool temp_driver_is_fan_on(void)
{
    portENTER_CRITICAL(&s_mux);
    bool v = s_fanCmd;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

float temp_driver_get_point(void)
{
    portENTER_CRITICAL(&s_mux);
    float v = s_setpointC;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

void temp_driver_set_point(float celsius)
{
    if (celsius < 0.0f)  celsius = 0.0f;
    if (celsius > 60.0f) celsius = 60.0f;

    portENTER_CRITICAL(&s_mux);
    s_setpointC = celsius;
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGI(TAG, "Setpoint atualizado: %.1fC", celsius);
}
