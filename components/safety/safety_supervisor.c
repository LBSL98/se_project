#include "safety_supervisor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "motors_driver.h"   // motors_driver_set_speed()


static const char *TAG_SAFE = "safety";

/* --------------------------------------------------------------------------
 *  Configuração de alocação estática
 * -------------------------------------------------------------------------- */

#define CMD_QUEUE_LEN           10      // Tamanho da fila de comandos
#define SAFETY_TASK_STACK_SIZE  4096    // Profundidade da pilha (words)

static uint8_t      s_queue_storage[CMD_QUEUE_LEN * sizeof(control_cmd_t)];
static StaticQueue_t s_queue_struct;
static QueueHandle_t s_control_queue = NULL;

static StackType_t  s_task_stack[SAFETY_TASK_STACK_SIZE];
static StaticTask_t s_task_tcb;

/* --------------------------------------------------------------------------
 *  Lógica de segurança
 * -------------------------------------------------------------------------- */

/**
 * @brief Aplica regras de segurança à velocidade de referência.
 *
 * @param requested_speed Velocidade desejada pela lógica de controle (-100..100).
 * @return Velocidade segura após aplicação das regras.
 */
static int8_t apply_safety_rules(int8_t requested_speed)
{
    /* 1. Clamping absoluto */
    if (requested_speed > 100) {
        requested_speed = 100;
    } else if (requested_speed < -100) {
        requested_speed = -100;
    }

    /* 2. Limite global de segurança (ex.: modo "criança" / teste) */
    const int8_t GLOBAL_SPEED_LIMIT = 80;

    if (requested_speed > GLOBAL_SPEED_LIMIT) {
        ESP_LOGW(TAG_SAFE,
                 "Clamp de segurança: %d%% -> %d%%",
                 requested_speed, GLOBAL_SPEED_LIMIT);
        return GLOBAL_SPEED_LIMIT;
    }

    /* 3. (Futuro) Regras baseadas em sensores
     *
     * Exemplo:
     *
     *  if (sensors_get_distance_cm() < 20) {
     *      ESP_LOGW(TAG_SAFE, "Obstáculo próximo! Forçando speed = 0.");
     *      return 0;
     *  }
     */

    return requested_speed;
}

/* --------------------------------------------------------------------------
 *  Task Gatekeeper (Safety)
 * -------------------------------------------------------------------------- */

static void safety_task_loop(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG_SAFE, "Supervisor de segurança iniciado (Gatekeeper).");

    control_cmd_t cmd;
    const TickType_t WATCHDOG_TIMEOUT = pdMS_TO_TICKS(500);  // 500 ms

    while (1) {
        /* Espera por um comando novo ou timeout do watchdog */
        if (xQueueReceive(s_control_queue, &cmd, WATCHDOG_TIMEOUT) == pdTRUE) {

            /* Verificação básica de latência */
            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            uint32_t age_ms = now_ms - cmd.timestamp_ms;  // unsigned lida bem com overflow

            if (age_ms > 200U) {
                ESP_LOGW(TAG_SAFE,
                         "Comando descartado por latência excessiva (%lu ms)",
                         (unsigned long)age_ms);
                continue;
            }

            /* Aplica regras e atua no driver de motores */
            int8_t safe_speed = apply_safety_rules(cmd.speed_percent);
                        int speed = (int)safe_speed;
            if (speed < 0)   speed = 0;
            if (speed > 100) speed = 100;

            // aplica a mesma velocidade para TODOS os 4 motores, para frente
            for (int m = 0; m < 4; ++m) {
                motors_driver_set_speed((uint8_t)m, speed, MOTOR_DIR_FORWARD);
            }

        }
        else {
            /* WATCHDOG DISPARADO:
             * Nenhum comando chegou dentro do intervalo.
             * A ação segura é parar os motores.
             */
            ESP_LOGE(TAG_SAFE,
                     "TIMEOUT: Control Task silenciosa! Parando motores (fail-safe).");
                        // nova função para parar tudo com segurança
            motors_driver_stop_all();

        }
    }
}

/* --------------------------------------------------------------------------
 *  Inicialização pública
 * -------------------------------------------------------------------------- */

void safety_supervisor_init(void)
{
    /* Cria fila estática */
    s_control_queue = xQueueCreateStatic(
        CMD_QUEUE_LEN,
        sizeof(control_cmd_t),
        s_queue_storage,
        &s_queue_struct
    );

    if (s_control_queue == NULL) {
        ESP_LOGE(TAG_SAFE, "Falha ao criar fila estática de comandos de segurança!");
        return;
    }

    /* Cria task estática pinada no Core 1 com alta prioridade */
    TaskHandle_t handle = xTaskCreateStaticPinnedToCore(
        safety_task_loop,
        "SafetyTask",
        SAFETY_TASK_STACK_SIZE,
        NULL,
        10,               // Prioridade (maior que a de controle)
        s_task_stack,
        &s_task_tcb,
        1                 // Core 1
    );

    if (handle == NULL) {
        ESP_LOGE(TAG_SAFE, "Falha ao criar SafetyTask estática!");
        return;
    }

    ESP_LOGI(TAG_SAFE, "safety_supervisor_init(): fila e task criadas com sucesso.");
}

BaseType_t safety_post_command(const control_cmd_t *cmd)
{
    if ((s_control_queue == NULL) || (cmd == NULL)) {
        return pdFALSE;
    }

    /* Envio não bloqueante. Se a fila estiver cheia, simplesmente falha. */
    return xQueueSend(s_control_queue, cmd, 0);
}
