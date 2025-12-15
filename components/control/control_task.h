#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa a task de controle de alto nível.
 *
 * Cria a ControlTask pinada no core 0. A task é responsável por
 * gerar comandos de velocidade e enviá-los ao safety_supervisor.
 */
void control_task_init(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_TASK_H
