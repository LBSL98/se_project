#include "sensors.h"

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "driver/adc.h"   
#include "driver/gpio.h"
#include "esp_log.h"

#define LDR_ADC_CHANNEL      ADC1_CHANNEL_4   // GPIO32
#define LDR_ADC_WIDTH        ADC_WIDTH_BIT_12
#define LDR_ADC_ATTEN        ADC_ATTEN_DB_12  // ~0..3.3V

#define LDR_LED_GPIO         GPIO_NUM_2

#define LDR_DELTA_DEFAULT    80               // margem pra escurecer
#define LDR_CALIB_SAMPLES    200              // 200 amostras de calibração
#define LDR_CALIB_DELAY_MS   10               // 10 ms entre amostras
#define LDR_SAMPLE_PERIOD_MS 500              // leitura a cada 0,5 s

static const char *TAG = "sensors";

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static int  s_ldr_ambient   = 0;                 // média ambiente calibrada
static int  s_ldr_threshold = 0;                 // limiar (ambient - delta)
static int  s_ldr_delta     = LDR_DELTA_DEFAULT; // ajustável via Blynk (6B)
static int  s_ldr_last_raw  = 0;                 // última leitura ADC
static bool s_ldr_led_on    = false;             // estado atual do LED (para Blynk)

static int ldr_read_raw(void)
{
    int raw = adc1_get_raw(LDR_ADC_CHANNEL);

    if (raw < 0)    raw = 0;
    if (raw > 4095) raw = 4095;

    return raw;
}

static void recompute_threshold_locked(void)
{
    int thr = s_ldr_ambient - s_ldr_delta;
    if (thr < 0)    thr = 0;
    if (thr > 4095) thr = 4095;
    s_ldr_threshold = thr;
}

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

    int ambient = (n > 0) ? (int)(sum / n) : 1000;

    int delta, thr;
    portENTER_CRITICAL(&s_mux);
    s_ldr_ambient = ambient;
    recompute_threshold_locked();
    delta = s_ldr_delta;
    thr   = s_ldr_threshold;
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGI(TAG, "Ambient=%d  DELTA=%d  THR=%d", ambient, delta, thr);
}

static void ldr_task(void *arg)
{
    (void)arg;

    while (1) {
        int raw = ldr_read_raw();

        int  thr;
        bool led_on;

        portENTER_CRITICAL(&s_mux);
        s_ldr_last_raw = raw;
        thr = s_ldr_threshold;
        led_on = (raw < thr);   // acende quando escurece em relação ao ambiente
        s_ldr_led_on = led_on;
        portEXIT_CRITICAL(&s_mux);

        gpio_set_level(LDR_LED_GPIO, led_on ? 1 : 0);

        ESP_LOGI(TAG, "LDR=%d  THR=%d  LED=%s", raw, thr, led_on ? "ON" : "OFF");
        vTaskDelay(pdMS_TO_TICKS(LDR_SAMPLE_PERIOD_MS));
    }
}

esp_err_t sensors_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << LDR_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LDR_LED_GPIO, 0);

    adc1_config_width(LDR_ADC_WIDTH);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, LDR_ADC_ATTEN);

    ldr_calibrate();

    // Tarefa de leitura
    BaseType_t res = xTaskCreate(ldr_task, "ldr_task", 4096, NULL, 5, NULL);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar ldr_task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

int sensors_get_ldr_raw(void)
{
    portENTER_CRITICAL(&s_mux);
    int v = s_ldr_last_raw;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

int sensors_get_ldr_threshold(void)
{
    portENTER_CRITICAL(&s_mux);
    int v = s_ldr_threshold;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

void sensors_set_ldr_delta(int delta)
{
    if (delta < 0)    delta = 0;
    if (delta > 4095) delta = 4095;

    int ambient, thr;

    portENTER_CRITICAL(&s_mux);
    s_ldr_delta = delta;
    recompute_threshold_locked();
    ambient = s_ldr_ambient;
    thr     = s_ldr_threshold;
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGI(TAG, "LDR_DELTA atualizado: %d (Ambient=%d -> THR=%d)", delta, ambient, thr);
}

int sensors_get_ldr_delta(void)
{
    portENTER_CRITICAL(&s_mux);
    int v = s_ldr_delta;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

bool sensors_is_ldr_led_on(void)
{
    portENTER_CRITICAL(&s_mux);
    bool v = s_ldr_led_on;
    portEXIT_CRITICAL(&s_mux);
    return v;
}
