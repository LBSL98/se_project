#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Inicializa os sensores.
 * Depois a gente separa sensores lentos/rápidos de forma decente.
 */
void sensors_init(void);

#ifdef __cplusplus
}
#endif

#endif // SENSORS_H
