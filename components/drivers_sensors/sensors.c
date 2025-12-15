#include "sensors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"   // legado, mas ok por enquanto
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdbool.h>

#define LDR_ADC_CHANNEL      ADC1_CHANNEL_4   // GPIO32
#define LDR_ADC_WIDTH        ADC_WIDTH_BIT_12
#define LDR_ADC_ATTEN        ADC_ATTEN_DB_11  // ~0..3.3V

// LED só para debug visual (use um pino livre, aqui GPIO2)
#define LDR_LED_GPIO         GPIO_NUM_2

#define LDR_DELTA_DEFAULT    50               // margem pra escurecer
#define LDR_CALIB_SAMPLES    200              // 200 amostras de calibração
#define LDR_CALIB_DELAY_MS   10               // 10 ms entre amostras
#define LDR_SAMPLE_PERIOD_MS 500              // leitura a cada 0,5 s

static const char *TAG = "sensors";

static int s_ldr_threshold = 0;
static int s_ldr_delta     = LDR_DELTA_DEFAULT;
static int s_ldr_last_raw  = 0;

// Leitura bruta (0..4095) do ADC
static int ldr_read_raw(void)
{
    int raw = adc1_get_raw(LDR_ADC_CHANNEL);

    if (raw < 0)    raw = 0;
    if (raw > 4095) raw = 4095;

    return raw;
}

// Faz uma calibração simples: mede a luz ambiente e define um limiar abaixo dela.
// LED acende quando ficar MAIS ESCURO que o ambiente original.
static void ldr_calibrate(void)
{
    ESP_LOGI(TAG, "Calibrando LDR com %d amostras...", LDR_CALIB_SAMPLES);

    long sum = 0;
    int  n   = 0;

    for (int i = 0; i < LDR_CALIB_SAMPLES; i++) {
        int raw = ldr_read_raw();
        sum += raw;
        n++;
        vTaskDelay(pdMS_TO_TICKS(LDR_CALIB_DELAY_MS));
    }

    int ambient = (n > 0) ? (sum / n) : 1000;

    // Limiar = um pouco ABAIXO da média ambiente
    s_ldr_threshold = ambient - s_ldr_delta;
    if (s_ldr_threshold < 0) {
        s_ldr_threshold = 0;
    }

    ESP_LOGI(TAG, "Ambient=%d  DELTA=%d  THR=%d",
             ambient, s_ldr_delta, s_ldr_threshold);
}

static void ldr_task(void *arg)
{
    while (1) {
        int raw = ldr_read_raw();
        s_ldr_last_raw = raw;

        // Regra: se escureceu em relacao ao ambiente, acende o LED
        bool led_on = (raw < s_ldr_threshold);

        gpio_set_level(LDR_LED_GPIO, led_on ? 1 : 0);

        ESP_LOGI(TAG, "LDR=%d  THR=%d  LED=%s",
                 raw, s_ldr_threshold, led_on ? "ON" : "OFF");

        vTaskDelay(pdMS_TO_TICKS(LDR_SAMPLE_PERIOD_MS));
    }
}

esp_err_t sensors_init(void)
{
    // LED de debug
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << LDR_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LDR_LED_GPIO, 0);

    // ADC do LDR em GPIO32 / ADC1_CHANNEL_4
    adc1_config_width(LDR_ADC_WIDTH);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, LDR_ADC_ATTEN);

    // Calibra com a luz ambiente atual
    ldr_calibrate();

    // Cria tarefa de leitura periódica
    BaseType_t res = xTaskCreate(
        ldr_task,
        "ldr_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar ldr_task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

int sensors_get_ldr_raw(void)
{
    return s_ldr_last_raw;
}

int sensors_get_ldr_threshold(void)
{
    return s_ldr_threshold;
}
