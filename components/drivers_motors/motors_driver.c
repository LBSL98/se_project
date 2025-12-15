#include "motors_driver.h"
#include "hardware_map.h"

#include "driver/gpio.h"
#include "esp_log.h"

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
} motor_pins_t;

static const char *TAG = "motors_driver";

// Mapeia ID lógico -> pinos físicos
static const motor_pins_t motors[MOTOR_COUNT] = {
    [MOTOR_FRONT_LEFT]  = { PIN_A_IN1, PIN_A_IN2 },
    [MOTOR_FRONT_RIGHT] = { PIN_A_IN3, PIN_A_IN4 },
    [MOTOR_REAR_RIGHT]  = { PIN_B_IN1, PIN_B_IN2 },
    [MOTOR_REAR_LEFT]   = { PIN_B_IN3, PIN_B_IN4 },
};

static void set_pins_dir(gpio_num_t in1, gpio_num_t in2, int dir)
{
    switch (dir) {
        case MOTOR_DIR_FORWARD:
            gpio_set_level(in1, 1);
            gpio_set_level(in2, 0);
            break;
        case MOTOR_DIR_BACKWARD:
            gpio_set_level(in1, 0);
            gpio_set_level(in2, 1);
            break;
        default:  // STOP
            gpio_set_level(in1, 0);
            gpio_set_level(in2, 0);
            break;
    }
}

esp_err_t motors_driver_init(void)
{
    gpio_config_t io_conf = {0};

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask =
        (1ULL << PIN_A_IN1) | (1ULL << PIN_A_IN2) |
        (1ULL << PIN_A_IN3) | (1ULL << PIN_A_IN4) |
        (1ULL << PIN_B_IN1) | (1ULL << PIN_B_IN2) |
        (1ULL << PIN_B_IN3) | (1ULL << PIN_B_IN4);

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %d", err);
        return err;
    }

    motors_driver_stop_all();

    ESP_LOGI(TAG, "Motors driver initialized (GPIO only, no PWM).");
    return ESP_OK;
}

esp_err_t motors_driver_set_speed(uint8_t motor_id, int speed_pct, int dir)
{
    if (motor_id >= MOTOR_COUNT) {
        ESP_LOGE(TAG, "Invalid motor_id: %u", motor_id);
        return ESP_ERR_INVALID_ARG;
    }

    const motor_pins_t *mp = &motors[motor_id];

    if (speed_pct <= 0 || dir == MOTOR_DIR_STOP) {
        set_pins_dir(mp->in1, mp->in2, MOTOR_DIR_STOP);
        return ESP_OK;
    }

    if (dir != MOTOR_DIR_FORWARD && dir != MOTOR_DIR_BACKWARD) {
        ESP_LOGE(TAG, "Invalid dir: %d", dir);
        return ESP_ERR_INVALID_ARG;
    }

    // Sem PWM: qualquer speed_pct > 0 é "ligado direto"
    if (speed_pct > 100) {
        speed_pct = 100;
    }

    set_pins_dir(mp->in1, mp->in2, dir);
    return ESP_OK;
}

void motors_driver_stop_all(void)
{
    for (uint8_t m = 0; m < MOTOR_COUNT; ++m) {
        const motor_pins_t *mp = &motors[m];
        set_pins_dir(mp->in1, mp->in2, MOTOR_DIR_STOP);
    }
    ESP_LOGI(TAG, "All motors STOP.");
}

void motors_driver_stop(void)
{
    motors_driver_stop_all();
}

// Helpers internos
static esp_err_t set_all(int dir, int speed_pct)
{
    for (uint8_t m = 0; m < MOTOR_COUNT; ++m) {
        esp_err_t err = motors_driver_set_speed(m, speed_pct, dir);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

// Movimentos de alto nível:

esp_err_t motors_driver_move_forward(int speed_pct)
{
    ESP_LOGI(TAG, "Move FORWARD, speed=%d", speed_pct);
    return set_all(MOTOR_DIR_FORWARD, speed_pct);
}

esp_err_t motors_driver_move_backward(int speed_pct)
{
    ESP_LOGI(TAG, "Move BACKWARD, speed=%d", speed_pct);
    return set_all(MOTOR_DIR_BACKWARD, speed_pct);
}

esp_err_t motors_driver_turn_left(int speed_pct)
{
    ESP_LOGI(TAG, "Turn LEFT, speed=%d", speed_pct);

    esp_err_t err;

    // Lado esquerdo ré
    err = motors_driver_set_speed(MOTOR_FRONT_LEFT,  speed_pct, MOTOR_DIR_BACKWARD);
    if (err != ESP_OK) return err;
    err = motors_driver_set_speed(MOTOR_REAR_LEFT,   speed_pct, MOTOR_DIR_BACKWARD);
    if (err != ESP_OK) return err;

    // Lado direito frente
    err = motors_driver_set_speed(MOTOR_FRONT_RIGHT, speed_pct, MOTOR_DIR_FORWARD);
    if (err != ESP_OK) return err;
    err = motors_driver_set_speed(MOTOR_REAR_RIGHT,  speed_pct, MOTOR_DIR_FORWARD);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t motors_driver_turn_right(int speed_pct)
{
    ESP_LOGI(TAG, "Turn RIGHT, speed=%d", speed_pct);

    esp_err_t err;

    // Lado esquerdo frente
    err = motors_driver_set_speed(MOTOR_FRONT_LEFT,  speed_pct, MOTOR_DIR_FORWARD);
    if (err != ESP_OK) return err;
    err = motors_driver_set_speed(MOTOR_REAR_LEFT,   speed_pct, MOTOR_DIR_FORWARD);
    if (err != ESP_OK) return err;

    // Lado direito ré
    err = motors_driver_set_speed(MOTOR_FRONT_RIGHT, speed_pct, MOTOR_DIR_BACKWARD);
    if (err != ESP_OK) return err;
    err = motors_driver_set_speed(MOTOR_REAR_RIGHT,  speed_pct, MOTOR_DIR_BACKWARD);
    if (err != ESP_OK) return err;

    return ESP_OK;
}
