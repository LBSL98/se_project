#include "safety_supervisor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "motors_driver.h"

static const char *TAG_SAFE = "safety_supervisor";

/* Estrutura de comando que vem da control_task */
typedef struct {
    int speed_percent;
} safety_control_cmd_t;

/* Fila de comandos vinda da control_task */
static QueueHandle_t s_control_cmd_queue = NULL;

/* Regra simples de segurança:
 * - Limita velocidade máxima a 80%
 * - Clampa entre -100% e +100%
 * Aqui depois você coloca:
 *   - regras de timeout (watchdog)
 *   - leitura de sensores
 *   - modo emergência, etc.
 */
static int apply_safety_rules(int requested_speed)
{
    if (requested_speed > 100) {
        requested_speed = 100;
    } else if (requested_speed < -100) {
        requested_speed = -100;
    }

    int safe_speed = requested_speed;

    /* Exemplo de "freio de segurança": nada acima de 80% */
    if (safe_speed > 80) {
        ESP_LOGW(TAG_SAFE,
                 "Speed %d%% acima do limite! Reduzindo para 80%%.",
                 safe_speed);
        safe_speed = 80;
    }

    return safe_speed;
}

/* Task de segurança: Gatekeeper entre controle e motores */
static void safety_task(void *pvParameters)
{
    ESP_LOGI(TAG_SAFE, "safety_supervisor task iniciada (Gatekeeper).");

    safety_control_cmd_t cmd;

    while (1) {
        /* Espera por um comando da control_task, com timeout de 500 ms */
        if (xQueueReceive(s_control_cmd_queue,
                          &cmd,
                          pdMS_TO_TICKS(500)) == pdTRUE) {

            int safe_speed = apply_safety_rules(cmd.speed_percent);

            ESP_LOGI(TAG_SAFE,
                     "Comando recebido: %d%% -> Comando seguro: %d%%",
                     cmd.speed_percent,
                     safe_speed);

            /* Só a task de segurança fala diretamente com os motores */
            motors_driver_set_speed(safe_speed);
        } else {
            /* Timeout: aqui podemos implementar watchdog de comando.
             * Exemplo simples: depois de um tempo sem comandos, reduzir para 0%.
             * (Por enquanto, só loga.)
             */
            ESP_LOGD(TAG_SAFE, "Nenhum comando recebido no período (stub).");
        }
    }
}

void safety_supervisor_init(void)
{
    ESP_LOGI(TAG_SAFE, "Inicializando safety_supervisor...");

    /* Cria fila para receber comandos da control_task */
    s_control_cmd_queue = xQueueCreate(10, sizeof(safety_control_cmd_t));
    if (s_control_cmd_queue == NULL) {
        ESP_LOGE(TAG_SAFE, "Falha ao criar fila de comandos de controle!");
        return;
    }

    BaseType_t res = xTaskCreatePinnedToCore(
        safety_task,
        "safety_task",
        4096,
        NULL,
        5,      // prioridade mais alta que controle
        NULL,
        1       // core 1 (por enquanto)
    );
    if (res != pdPASS) {
        ESP_LOGE(TAG_SAFE, "Falha ao criar safety_task");
    }
}

/* Função chamada pela control_task para publicar comandos */
BaseType_t safety_post_control_command(int speed_percent)
{
    if (s_control_cmd_queue == NULL) {
        ESP_LOGE(TAG_SAFE,
                 "Fila de comandos ainda não criada! Ignorando comando %d%%",
                 speed_percent);
        return pdFAIL;
    }

    safety_control_cmd_t cmd = {
        .speed_percent = speed_percent
    };

    /* Envia sem bloquear (0 ticks). Se a fila estiver cheia, falha. */
    BaseType_t res = xQueueSend(s_control_cmd_queue, &cmd, 0);
    if (res != pdPASS) {
        ESP_LOGW(TAG_SAFE,
                 "Fila cheia, comando %d%% descartado.",
                 speed_percent);
    }

    return res;
}
