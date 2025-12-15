#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "wifi_comm.h"
#include "motors_driver.h"
#include "sensors.h"
#include "safety_supervisor.h"
#include "control_task.h"

static const char *TAG = "app_main";

static void motors_demo_task(void *pv)
{
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

void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main...");

    wifi_comm_init();

    ESP_ERROR_CHECK(motors_driver_init());
    ESP_ERROR_CHECK(sensors_init());

    safety_supervisor_init();
    control_task_init();

    xTaskCreatePinnedToCore(
        motors_demo_task,
        "motors_demo",
        4096,
        NULL,
        4,
        NULL,
        tskNO_AFFINITY
    );

    ESP_LOGI(TAG, "Inicializacao concluida. Tasks em execucao.");
}
