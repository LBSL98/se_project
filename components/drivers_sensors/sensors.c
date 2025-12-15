#include "sensors.h"
#include "esp_log.h"

static const char *TAG_SENS = "sensors";

void sensors_init(void)
{
    // TODO: configurar ADC, GPIOs, I2C etc.
    ESP_LOGI(TAG_SENS, "sensors_init(): sensores inicializados (stub).");
}
