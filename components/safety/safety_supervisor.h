#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"  // BaseType_t

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Data Transfer Object (DTO) para comandos de controle.
 * Exposto no header para que control_task e safety_supervisor
 * concordem explicitamente sobre o formato da mensagem.
 */
typedef struct {
    uint32_t timestamp_ms;   /* Momento em que o comando foi gerado */
    int8_t   speed_percent;  /* Velocidade desejada: -100 a +100 (%) */
} control_cmd_t;

/**
 * @brief Inicializa o supervisor de segurança.
 *        Cria fila estática e task de segurança (Gatekeeper).
 */
void safety_supervisor_init(void);

/**
 * @brief Envia um comando para a fila de segurança.
 *
 * @param cmd Ponteiro para o comando preenchido.
 * @return pdTRUE se sucesso, pdFALSE se fila cheia ou não inicializada.
 */
BaseType_t safety_post_command(const control_cmd_t *cmd);

#ifdef __cplusplus
}
#endif

#endif // SAFETY_SUPERVISOR_H
