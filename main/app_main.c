#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "wifi_comm.h"
#include "motors_driver.h"
#include "sensors.h"

#include "temp_driver.h"
#include "ultrasonic_driver.h"

#include "safety_supervisor.h"
#include "control_task.h"

#include "comm_blynk.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main...");

    wifi_comm_init();

    ESP_ERROR_CHECK(motors_driver_init());
    sensors_init();
    ESP_ERROR_CHECK(temp_driver_init(17, 16, 25.0f));


    // HC-SR04: TRIG=13, ECHO=34, trava se < 20cm
    ultrasonic_driver_init(GPIO_NUM_22, GPIO_NUM_34, 20.0f);

    safety_supervisor_init();
    control_task_init();

    esp_err_t err = comm_blynk_init();
    if (err != ESP_OK) {
        ESP_LOGW("app_main", "Blynk desabilitado: %s", esp_err_to_name(err));
    }


    ESP_LOGI(TAG, "Inicializacao concluida. Tasks em execucao.");
}
