#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "wifi_comm.h"
#include "motors_driver.h"
#include "sensors.h"
#include "safety_supervisor.h"
#include "control_task.h"

static const char *TAG_MAIN = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "Starting app_main...");

    // 1) Comunicação (Wi-Fi station), reaproveitando o exemplo
    wifi_comm_init();

    // 2) Motores: inicializa hardware (por enquanto só log)
    motors_driver_init();

    // 3) Sensores: inicializa hardware (por enquanto só log)
    sensors_init();

    // 4) Segurança: task supervisor (prioridade alta)
    safety_supervisor_init();

    // 5) Controle: task que vai gerar comandos (e futuramente o bug)
    control_task_init();

    // app_main não precisa de loop; as tasks agora cuidam de tudo.
    ESP_LOGI(TAG_MAIN, "Inicialização concluída. Tasks em execução.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
