#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "wifi_comm.h"
#include "motors_driver.h"
#include "sensors.h"

#include "safety_supervisor.h"
#include "control_task.h"

#include "temp_driver.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main...");

    // 1) Wi-Fi (se já funciona, mantém)
    wifi_comm_init();

    // 2) Motores + sensores base
    ESP_ERROR_CHECK(motors_driver_init());
    sensors_init();

    // 3) Safety primeiro (cria fila/infra que o control usa)
    safety_supervisor_init();

    // 4) Control depois (vai alimentar o safety e evitar timeout)
    control_task_init();

    // 5) Temperatura por último: roda em paralelo, sem comandar motor
    // (não use ESP_ERROR_CHECK se temp_driver_init() for void)
    temp_driver_init();

    ESP_LOGI(TAG, "Inicializacao concluida. Sistema em execucao.");
}
