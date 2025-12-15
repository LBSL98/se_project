#ifndef MOTORS_DRIVER_H
#define MOTORS_DRIVER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Direções
#define MOTOR_DIR_STOP       0
#define MOTOR_DIR_FORWARD    1
#define MOTOR_DIR_BACKWARD  -1

// IDs dos 4 motores
#define MOTOR_FRONT_LEFT     0
#define MOTOR_FRONT_RIGHT    1
#define MOTOR_REAR_RIGHT     2
#define MOTOR_REAR_LEFT      3
#define MOTOR_COUNT          4

// Inicialização do driver (configura GPIOs)
esp_err_t motors_driver_init(void);

// Define "velocidade" e direção de UM motor.
// Aqui, qualquer speed_pct > 0 vira simplesmente ON (sem PWM).
esp_err_t motors_driver_set_speed(uint8_t motor_id, int speed_pct, int dir);

// Para todos os motores
void      motors_driver_stop_all(void);
void      motors_driver_stop(void);  // alias, só chama stop_all

// Movimentos de alto nível (todos os motores):
esp_err_t motors_driver_move_forward(int speed_pct);
esp_err_t motors_driver_move_backward(int speed_pct);
esp_err_t motors_driver_turn_left(int speed_pct);
esp_err_t motors_driver_turn_right(int speed_pct);

#ifdef __cplusplus
}
#endif

#endif // MOTORS_DRIVER_H
