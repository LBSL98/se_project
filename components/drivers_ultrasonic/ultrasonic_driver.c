#include "ultrasonic_driver.h"

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "ultrasonic";

static gpio_num_t s_trig = GPIO_NUM_NC;
static gpio_num_t s_echo = GPIO_NUM_NC;

static float s_threshold_cm = 20.0f;
static float s_clear_margin_cm = 8.0f;   // histerese para liberar (thr + margin)

static float s_last_valid_cm = -1.0f;
static int64_t s_last_valid_us = 0;

static bool s_obstacle = false;
static uint8_t s_near_cnt = 0;
static uint8_t s_far_cnt  = 0;

static bool s_inited = false;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// timeouts (us)
#define WAIT_RISE_TIMEOUT_US   (15000)
#define WAIT_FALL_TIMEOUT_US   (15000)

// período da task
#define ULTRA_PERIOD_MS        (120)

// debounce
#define OBSTACLE_ASSERT_SAMPLES  2
#define OBSTACLE_CLEAR_SAMPLES   3

static inline float pulse_us_to_cm(int64_t pulse_us)
{
    return (float)pulse_us * 0.0343f * 0.5f;
}

static float read_distance_once_cm(void)
{
    if (!s_inited) return -1.0f;

    gpio_set_level(s_trig, 0);
    esp_rom_delay_us(2);

    gpio_set_level(s_trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s_trig, 0);

    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(s_echo) == 0) {
        if ((esp_timer_get_time() - t0) > WAIT_RISE_TIMEOUT_US) return -1.0f;
    }

    int64_t t_start = esp_timer_get_time();
    while (gpio_get_level(s_echo) == 1) {
        if ((esp_timer_get_time() - t_start) > WAIT_FALL_TIMEOUT_US) return -1.0f;
    }

    int64_t pulse_us = esp_timer_get_time() - t_start;
    if (pulse_us <= 0) return -1.0f;

    float cm = pulse_us_to_cm(pulse_us);
    if (cm <= 0.5f) return -1.0f;     // descarta leituras “quase zero” típicas de ruído
    if (cm > 300.0f) return -1.0f;    // descarta muito longe

    return cm;
}

static void sort_small(float *a, int n)
{
    // insertion sort (n pequeno)
    for (int i = 1; i < n; i++) {
        float key = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > key) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
}

static bool compute_filtered_cm(float *out_cm)
{
    // 5 amostras, pega mediana (reduz falso positivo)
    float v[5];
    int n = 0;

    for (int i = 0; i < 5; i++) {
        float cm = read_distance_once_cm();
        if (cm > 0.0f) v[n++] = cm;
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    if (n == 0) return false;

    sort_small(v, n);

    float cm_f;
    if (n >= 3) {
        cm_f = v[n / 2];            // mediana
    } else {
        cm_f = 0.5f * (v[0] + v[n - 1]); // média simples se só 1-2 válidas
    }

    *out_cm = cm_f;
    return true;
}

static void ultrasonic_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Task started (TRIG=%d ECHO=%d thr=%.1fcm)",
             (int)s_trig, (int)s_echo, s_threshold_cm);

    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        float cm = -1.0f;
        bool ok = compute_filtered_cm(&cm);

        portENTER_CRITICAL(&s_mux);
        float thr = s_threshold_cm;
        float clr = s_threshold_cm + s_clear_margin_cm;

        if (ok) {
            s_last_valid_cm = cm;
            s_last_valid_us = esp_timer_get_time();

            // debounce com histerese
            if (!s_obstacle) {
                if (cm < thr) {
                    if (s_near_cnt < 255) s_near_cnt++;
                } else {
                    s_near_cnt = 0;
                }
                if (s_near_cnt >= OBSTACLE_ASSERT_SAMPLES) {
                    s_obstacle = true;
                    s_far_cnt = 0;
                }
            } else {
                if (cm > clr) {
                    if (s_far_cnt < 255) s_far_cnt++;
                } else {
                    s_far_cnt = 0;
                }
                if (s_far_cnt >= OBSTACLE_CLEAR_SAMPLES) {
                    s_obstacle = false;
                    s_near_cnt = 0;
                }
            }
        }
        // Se não ok: NÃO muda o estado (evita “para/anda” por leitura inválida)
        portEXIT_CRITICAL(&s_mux);

        // log leve
        static uint32_t c = 0;
        c++;
        if ((c % 10) == 0) {
            portENTER_CRITICAL(&s_mux);
            float d = s_last_valid_cm;
            bool  o = s_obstacle;
            portEXIT_CRITICAL(&s_mux);
            ESP_LOGI(TAG, "last=%.1fcm obstacle=%s", d, o ? "YES" : "NO");
        }

        vTaskDelay(pdMS_TO_TICKS(ULTRA_PERIOD_MS));
    }
}

void ultrasonic_driver_init(gpio_num_t trig_gpio, gpio_num_t echo_gpio, float threshold_cm)
{
    s_trig = trig_gpio;
    s_echo = echo_gpio;

    if (threshold_cm < 1.0f) threshold_cm = 1.0f;
    s_threshold_cm = threshold_cm;

    gpio_config_t trig_conf = {0};
    trig_conf.pin_bit_mask = (1ULL << s_trig);
    trig_conf.mode = GPIO_MODE_OUTPUT;
    trig_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&trig_conf);
    gpio_set_level(s_trig, 0);

    gpio_config_t echo_conf = {0};
    echo_conf.pin_bit_mask = (1ULL << s_echo);
    echo_conf.mode = GPIO_MODE_INPUT;
    echo_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&echo_conf);

    portENTER_CRITICAL(&s_mux);
    s_last_valid_cm = -1.0f;
    s_last_valid_us = 0;
    s_obstacle = false;
    s_near_cnt = 0;
    s_far_cnt  = 0;
    portEXIT_CRITICAL(&s_mux);

    s_inited = true;

    xTaskCreatePinnedToCore(ultrasonic_task, "ultrasonic_task", 4096, NULL, 2, NULL, 0);
}

void ultrasonic_driver_set_threshold_cm(float threshold_cm)
{
    if (threshold_cm < 1.0f) threshold_cm = 1.0f;
    portENTER_CRITICAL(&s_mux);
    s_threshold_cm = threshold_cm;
    portEXIT_CRITICAL(&s_mux);
}

float ultrasonic_driver_get_distance_cm(void)
{
    portENTER_CRITICAL(&s_mux);
    float v = s_last_valid_cm;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

bool ultrasonic_driver_is_obstacle_detected(void)
{
    portENTER_CRITICAL(&s_mux);
    bool v = s_obstacle;
    portEXIT_CRITICAL(&s_mux);
    return v;
}
