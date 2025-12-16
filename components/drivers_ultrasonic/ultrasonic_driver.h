#ifndef ULTRASONIC_DRIVER_H
#define ULTRASONIC_DRIVER_H

#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Inicializa o driver do HC-SR04 e cria a task de medição.
 *
 * @param trig_gpio GPIO de TRIG (output)
 * @param echo_gpio GPIO de ECHO (input) - recomendado usar 34/35
 * @param threshold_cm Distância em cm para travar motores (ex: 20.0f)
 */
void ultrasonic_driver_init(gpio_num_t trig_gpio, gpio_num_t echo_gpio, float threshold_cm);

/** Atualiza o limiar de obstáculo (cm). */
void ultrasonic_driver_set_threshold_cm(float threshold_cm);

/** Retorna a última distância válida (cm). Retorna < 0 se inválida. */
float ultrasonic_driver_get_distance_cm(void);

/** Retorna true se o driver considera "obstáculo detectado". */
bool ultrasonic_driver_is_obstacle_detected(void);

#ifdef __cplusplus
}
#endif

#endif // ULTRASONIC_DRIVER_H
