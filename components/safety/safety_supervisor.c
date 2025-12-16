#include "safety_supervisor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "motors_driver.h"

#include "ultrasonic_driver.h"  

static const char *TAG_SAFE = "safety";


#define CMD_QUEUE_LEN           10
#define SAFETY_TASK_STACK_SIZE  4096

static uint8_t       s_queue_storage[CMD_QUEUE_LEN * sizeof(control_cmd_t)];
static StaticQueue_t s_queue_struct;
static QueueHandle_t s_control_queue = NULL;

static StackType_t  s_task_stack[SAFETY_TASK_STACK_SIZE];
static StaticTask_t s_task_tcb;

static int8_t apply_safety_rules(int8_t requested_speed)
{
    if (requested_speed > 100) requested_speed = 100;
    if (requested_speed < -100) requested_speed = -100;

    const int8_t GLOBAL_SPEED_LIMIT = 80;
    if (requested_speed > GLOBAL_SPEED_LIMIT) return GLOBAL_SPEED_LIMIT;
    if (requested_speed < -GLOBAL_SPEED_LIMIT) return -GLOBAL_SPEED_LIMIT;

    if (ultrasonic_driver_is_obstacle_detected()) {
        static uint32_t last_log_ms = 0;
        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if ((now_ms - last_log_ms) > 500U) {
            float d = ultrasonic_driver_get_distance_cm();
            ESP_LOGW(TAG_SAFE, "Obstaculo detectado (dist=%.1fcm). Forcando STOP.", d);
            last_log_ms = now_ms;
        }
        return 0;
    }

    return requested_speed;
}

static void safety_task_loop(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG_SAFE, "Supervisor de seguranca iniciado (Gatekeeper).");

    control_cmd_t cmd;
    const TickType_t WATCHDOG_TIMEOUT = pdMS_TO_TICKS(500);

    while (1) {
        if (xQueueReceive(s_control_queue, &cmd, WATCHDOG_TIMEOUT) == pdTRUE) {

            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            uint32_t age_ms = now_ms - cmd.timestamp_ms;

            if (age_ms > 200U) {
                ESP_LOGW(TAG_SAFE, "Comando descartado por latencia (%lu ms)",
                         (unsigned long)age_ms);
                continue;
            }

            int8_t safe_speed = apply_safety_rules(cmd.speed_percent);

            int speed = (int)safe_speed;
            if (speed < 0)   speed = 0;
            if (speed > 100) speed = 100;

           

        } else {
            ESP_LOGE(TAG_SAFE, "TIMEOUT: Control Task silenciosa! Parando motores.");
            motors_driver_stop_all();
        }
    }
}

void safety_supervisor_init(void)
{
    s_control_queue = xQueueCreateStatic(
        CMD_QUEUE_LEN,
        sizeof(control_cmd_t),
        s_queue_storage,
        &s_queue_struct
    );

    if (s_control_queue == NULL) {
        ESP_LOGE(TAG_SAFE, "Falha ao criar fila estatica de comandos!");
        return;
    }

    TaskHandle_t handle = xTaskCreateStaticPinnedToCore(
        safety_task_loop,
        "SafetyTask",
        SAFETY_TASK_STACK_SIZE,
        NULL,
        10,
        s_task_stack,
        &s_task_tcb,
        1
    );

    if (handle == NULL) {
        ESP_LOGE(TAG_SAFE, "Falha ao criar SafetyTask!");
        return;
    }

    ESP_LOGI(TAG_SAFE, "safety_supervisor_init(): OK.");
}

BaseType_t safety_post_command(const control_cmd_t *cmd)
{
    if ((s_control_queue == NULL) || (cmd == NULL)) {
        return pdFALSE;
    }
    return xQueueSend(s_control_queue, cmd, 0);
}
