#include "motors_driver.h"
#include "esp_log.h"

static const char *TAG_MOTORS = "motors_driver";

void motors_driver_init(void)
{
    // TODO: configurar GPIOs, timers, PWM (LEDC/MCPWM) etc.
    ESP_LOGI(TAG_MOTORS, "motors_driver_init(): motores inicializados (stub).");
}

void motors_driver_set_speed(int speed_percent)
{
    // Clampa valor só pra evitar besteira
    if (speed_percent > 100) speed_percent = 100;
    if (speed_percent < -100) speed_percent = -100;

    // TODO: traduzir isso para comandos reais de PWM para cada roda.
    ESP_LOGI(TAG_MOTORS,
             "motors_driver_set_speed(): speed=%d%% (stub, apenas log)",
             speed_percent);
}
