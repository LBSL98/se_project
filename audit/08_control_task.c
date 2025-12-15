#include "control_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "safety_supervisor.h"

static const char *TAG_CTRL = "control_task";

static void control_task(void *pvParameters)
{
    ESP_LOGI(TAG_CTRL, "control_task iniciada (stub).");

    int fake_cmd = 0;

    while (1) {
        /* Exemplo de "bug": gera comandos que podem passar de 100% */
        fake_cmd = (fake_cmd + 10) % 120; // 0,10,20,...,110

        ESP_LOGI(TAG_CTRL,
                 "Gerando comando de velocidade (bug/intent): %d",
                 fake_cmd);

        /* Em vez de falar com motores, manda intenção para o supervisor */
        BaseType_t res = safety_post_control_command(fake_cmd);
        if (res != pdPASS) {
            ESP_LOGW(TAG_CTRL,
                     "Falha ao enviar comando %d%% para o supervisor (fila cheia?).",
                     fake_cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void control_task_init(void)
{
    ESP_LOGI(TAG_CTRL, "Inicializando control_task...");
    BaseType_t res = xTaskCreatePinnedToCore(
        control_task,
        "control_task",
        4096,
        NULL,
        4,      // prioridade abaixo da safety_task
        NULL,
        1       // Core 1 (por enquanto)
    );
    if (res != pdPASS) {
        ESP_LOGE(TAG_CTRL, "Falha ao criar control_task");
    }
}
