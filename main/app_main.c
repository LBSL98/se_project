#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"

#include "wifi_comm.h"
#include "motors_driver.h"
#include "sensors.h"
#include "safety_supervisor.h"
#include "control_task.h"

static const char *TAG = "app_main";

/* ===================== LDR CONFIG ===================== */

// LDR no GPIO36 → ADC1_CHANNEL_0 no ESP32
#define LDR_ADC_CHANNEL   ADC1_CHANNEL_4   // GPIO32
#define LDR_LED_GPIO      GPIO_NUM_2       // LED controlado pelo LDR

#define LDR_HYST          20               // histerese

static int  ldr_delta      = 50;
static int  ldr_ambient    = 0;
static int  ldr_threshold  = 0;
static bool ldr_led_state  = false;

/* ===================== Funções LDR ===================== */

static int ldr_read_raw(void)
{
    // Retorna valor bruto 0..4095
    return adc1_get_raw(LDR_ADC_CHANNEL);
}

/* ---------- Task que lê o LDR e liga/desliga o LED ---------- */

static void ldr_task(void *pvParameters)
{
    int v;

    while (1) {
        v = ldr_read_raw();

        // Escuro => valor sobe
        if (!ldr_led_state && v > (ldr_threshold + LDR_HYST)) {
            ldr_led_state = true;
            gpio_set_level(LDR_LED_GPIO, 1);
        } else if (ldr_led_state && v < (ldr_threshold - LDR_HYST)) {
            ldr_led_state = false;
            gpio_set_level(LDR_LED_GPIO, 0);
        }

        printf("LDR=%d  THR=%d  LED=%s\n",
               v, ldr_threshold, ldr_led_state ? "ON" : "OFF");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---------- Task que lê '+' e '-' da serial e ajusta DELTA ---------- */

static void ldr_serial_task(void *pvParameters)
{
    char c;

    while (1) {
        c = getchar();   // lê do terminal (idf.py monitor)

        if (c == '+') {
            ldr_delta += 50;
        } else if (c == '-') {
            ldr_delta -= 50;
            if (ldr_delta < 0) {
                ldr_delta = 0;
            }
        }

        ldr_threshold = ldr_ambient + ldr_delta;

        printf("DELTA=%d  THR=%d\n", ldr_delta, ldr_threshold);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------- Inicialização completa do LDR ---------- */

static void ldr_init(void)
{
    /* LED do LDR */
    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask  = (1ULL << LDR_LED_GPIO);
    io_conf.mode          = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type     = GPIO_INTR_DISABLE;

    gpio_config(&io_conf);
    gpio_set_level(LDR_LED_GPIO, 0);

    /* ADC (LDR) */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_12);

    /* Calibração do ambiente (2s) */
    ESP_LOGI(TAG, "Calibrando LDR (2s)...");

    long    sum = 0;
    int     n   = 0;
    int64_t t0  = esp_timer_get_time();

    while ((esp_timer_get_time() - t0) < 2000000) {  // 2s em us
        sum += ldr_read_raw();
        n++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ldr_ambient   = (n > 0) ? (sum / n) : 1000;
    ldr_threshold = ldr_ambient + ldr_delta;

    ESP_LOGI(TAG, "LDR ambient=%d  DELTA=%d  THR=%d",
             ldr_ambient, ldr_delta, ldr_threshold);
    printf("Use '+' e '-' no terminal para ajustar DELTA do LDR\n");

    /* Criação das tasks do LDR */
    xTaskCreate(ldr_task,        "ldr_task",        4096, NULL, 5, NULL);
    xTaskCreate(ldr_serial_task, "ldr_serial_task", 4096, NULL, 4, NULL);
}

/* ===================== app_main ===================== */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main...");

    // 1) Wi-Fi
    wifi_comm_init();

    // 2) Drivers / arquitetura existente
    ESP_ERROR_CHECK(motors_driver_init());
    sensors_init();
    safety_supervisor_init();
    control_task_init();

    // 3) LDR (GPIO36 + LED no GPIO2)
    ldr_init();

    ESP_LOGI(TAG, "Inicializacao concluida. Tasks em execucao.");

    // 4) Loop de teste dos motores (o mesmo que já funcionava)
    while (1) {
        ESP_LOGI(TAG, "Frente (2s)");
        motors_driver_move_forward(100);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Trás (2s)");
        motors_driver_move_backward(100);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Esquerda (2s)");
        motors_driver_turn_left(100);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Direita (2s)");
        motors_driver_turn_right(100);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Parado (2s)");
        motors_driver_stop_all();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
