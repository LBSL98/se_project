#include "control_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "safety_supervisor.h"   // control_cmd_t, safety_post_command()

static const char *TAG = "control_task";

/* -------------------------------------------------------------------------- *
 *  Configuração da task de controle
 * -------------------------------------------------------------------------- */

// Atenção: o safety_supervisor usa WATCHDOG_TIMEOUT = 500 ms.
// Então aqui mandamos comandos a cada 100 ms para NUNCA disparar o fail-safe
// em condição normal. O timeout só deve disparar depois que a task "morre".
#define CONTROL_TASK_STACK_SIZE      4096
#define CONTROL_TASK_PRIORITY        5
#define CONTROL_LOOP_PERIOD_MS       100     // 100 ms entre comandos (10 Hz)
#define CONTROL_NORMAL_ITERATIONS    10      // após 10 ciclos, simulamos travamento

static TaskHandle_t s_control_task_handle = NULL;

/* -------------------------------------------------------------------------- *
 *  Task de Controle
 * -------------------------------------------------------------------------- */

static void control_task_loop(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Inicializando control_task...");

    int8_t speed = 0;
    int    iteration = 0;

    ESP_LOGI(TAG,
             "control_task iniciada. Enviando comandos normais por %d iteracoes.",
             CONTROL_NORMAL_ITERATIONS);

    while (1) {
        iteration++;

        if (iteration <= CONTROL_NORMAL_ITERATIONS) {
            /* --------------------------------------------------------------
             *  Comportamento "normal":
             *  - Acelera em degraus de 10%: 10, 20, ..., 100
             *  - A cada 100 ms manda um novo comando pro safety_supervisor
             * -------------------------------------------------------------- */
            speed += 10;
            if (speed > 100) {
                speed = 10;
            }

            ESP_LOGI(TAG,
                     "Gerando comando de velocidade (bug/intent): %d (iter=%d)",
                     speed, iteration);

            control_cmd_t cmd = {
                .timestamp_ms  = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                .speed_percent = speed,
            };

            BaseType_t ok = safety_post_command(&cmd);
            if (ok != pdTRUE) {
                ESP_LOGW(TAG,
                         "Fila de safety cheia ou nao inicializada. Comando descartado.");
            }

            vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
        } else {
            /* --------------------------------------------------------------
             *  SIMULAÇÃO DE FALHA / TRAVAMENTO
             *
             *  Aqui simulamos o pior caso: a task "morre" e para de enviar
             *  comandos. A SafetyTask continua rodando, percebe que ninguém
             *  mais está publicando na fila por > 500 ms e:
             *      - Loga o TIMEOUT
             *      - Zera todos os motores (fail-safe)
             *
             *  Isso é exatamente o comportamento que você viu no log:
             *  TIMEOUTs periódicos + motores em 0%.
             * -------------------------------------------------------------- */
            ESP_LOGE(TAG,
                     "Simulando TRAVAMENTO: parando envio de comandos e encerrando control_task.");
            vTaskDelete(NULL);   // NUNCA retorna
        }
    }
}

/* -------------------------------------------------------------------------- *
 *  Inicialização pública
 * -------------------------------------------------------------------------- */

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
        0   // Core 0 (safety está no Core 1)
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar ControlTask!");
        s_control_task_handle = NULL;
    } else {
        ESP_LOGI(TAG, "ControlTask criada com sucesso.");
    }
}
