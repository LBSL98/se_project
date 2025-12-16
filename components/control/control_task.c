#include "control_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "safety_supervisor.h"   

static const char *TAG = "control_task";

#define CONTROL_TASK_STACK_SIZE    4096
#define CONTROL_TASK_PRIORITY      5
#define CONTROL_LOOP_PERIOD_MS     100     // 10 Hz (bem abaixo do watchdog de 500 ms)

// deixe 0 em produção
#define CONTROL_SIMULATE_CRASH     0
#define CONTROL_CRASH_AFTER_ITERS  10

static TaskHandle_t s_control_task_handle = NULL;

static void control_task_loop(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Inicializando control_task...");

    int8_t speed = 0;
    uint32_t iter = 0;

    while (1) {
        iter++;

        #if CONTROL_SIMULATE_CRASH
                if (iter > CONTROL_CRASH_AFTER_ITERS) {
                    ESP_LOGE(TAG, "Simulando travamento: encerrando control_task.");
                    vTaskDelete(NULL);
                }
        #endif

        // Exemplo simples: sobe 10..80 e volta
        speed += 10;
        if (speed > 80) speed = 10;

        control_cmd_t cmd = {
            .timestamp_ms  = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
            .speed_percent = speed,
        };

        BaseType_t ok = safety_post_command(&cmd);
        if (ok != pdTRUE) {
            ESP_LOGW(TAG, "Fila do safety cheia/indisponível. Comando descartado.");
        }

        if ((iter % 10) == 0) {
            ESP_LOGI(TAG, "cmd speed=%d%% (iter=%lu)", speed, (unsigned long)iter);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
    }
}

void control_task_init(void)
{
    ESP_LOGI(TAG, "control_task_init(): criando task de controle...");

    BaseType_t res = xTaskCreatePinnedToCore(
        control_task_loop,
        "ControlTask",
        CONTROL_TASK_STACK_SIZE,
        NULL,
        CONTROL_TASK_PRIORITY,
        &s_control_task_handle,
        0
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar ControlTask!");
        s_control_task_handle = NULL;
    } else {
        ESP_LOGI(TAG, "ControlTask criada com sucesso.");
    }
}
