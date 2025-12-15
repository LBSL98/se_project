#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "wifi_comm.h"          // wifi_comm_init()
#include "motors_driver.h"      // motors_driver_init(), motors_driver_move_*
#include "sensors.h"            // sensors_init()
#include "safety_supervisor.h"  // safety_supervisor_init()
#include "control_task.h"       // control_task_init()

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main...");

    // 1) Wi-Fi (o que já estava OK antes)
    wifi_comm_init();

    // 2) Motores + sensores (mantendo o ESP_ERROR_CHECK dos motores)
    ESP_ERROR_CHECK(motors_driver_init());
    sensors_init();

    // 3) Supervisor de segurança
    safety_supervisor_init();

    // 4) Task de controle principal
    control_task_init();

    ESP_LOGI(TAG, "Inicializacao concluida. Iniciando sequencia de teste dos motores...");

    // 5) Laço de teste dos motores (exatamente o que funcionava antes)
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
