#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Inicializa o Wi-Fi no modo station, usando as configs do sdkconfig.
 * Essa função encapsula o exemplo padrão wifi_station do ESP-IDF.
 */
void wifi_comm_init(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_COMM_H
