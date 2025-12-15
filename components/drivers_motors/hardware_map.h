#ifndef HARDWARE_MAP_H
#define HARDWARE_MAP_H

#include "driver/gpio.h"

/*
 * Ponte H A → eixo dianteiro
 * Ponte H B → eixo traseiro
 *
 * A e B estão "de frente uma pra outra".
 * Cada motor usa um par de pinos (IN1/IN2).
 */

// Ponte A – Dianteiro
#define PIN_A_IN1   GPIO_NUM_14  // Frente esquerda
#define PIN_A_IN2   GPIO_NUM_27
#define PIN_A_IN3   GPIO_NUM_26  // Frente direita
#define PIN_A_IN4   GPIO_NUM_25

// Ponte B – Traseiro
#define PIN_B_IN1   GPIO_NUM_18  // Trás direita
#define PIN_B_IN2   GPIO_NUM_19
#define PIN_B_IN3   GPIO_NUM_23  // Trás esquerda
#define PIN_B_IN4   GPIO_NUM_33

#endif // HARDWARE_MAP_H
