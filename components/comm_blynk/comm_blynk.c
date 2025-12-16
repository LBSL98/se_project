#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"

#include "temp_driver.h"
#include "sensors.h"
#include "motors_driver.h"


#if __has_include("comm_blynk_secrets.h")
#include "comm_blynk_secrets.h"
#endif

#ifndef BLYNK_DEVICE_TOKEN
#define BLYNK_DEVICE_TOKEN ""
#endif

#define DS_TEMPERATURE       "Temperature"
#define DS_HUMIDITY          "Humidity"
#define DS_LDR_RAW           "LDR_Raw"
#define DS_LDR_THR           "LDR_Thr"
#define DS_TEMP_SETPOINT     "Temp_SetC"
#define DS_LDR_DELTA         "LDR_Delta"
#define DS_TEMP_ALARM        "Temp_Alarm"   /* 0/1 */
#define DS_LDR_LED           "LDR_LED"      /* 0/1 */
#define DS_MOTOR_SPEED       "Motor_Speed"  /* 0..255 */

#define DS_M_FWD             "M_FWD"
#define DS_M_BACK            "M_BACK"
#define DS_M_LEFT            "M_LEFT"
#define DS_M_RIGHT           "M_RIGHT"
#define DS_M_STOP            "M_STOP"
#define DS_M_ROT_L           "M_ROT_L"
#define DS_M_ROT_R           "M_ROT_R"
#define DS_M_BRAKE           "M_BRAKE"       

static const char *TAG = "comm_blynk";

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;

static int s_speed_pct = 40;

static int clamp_i(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float clamp_f(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int slider255_to_pct(int v255) {
    v255 = clamp_i(v255, 0, 255);
    return (v255 * 100 + 127) / 255;
}

static void motors_apply(const char *ds_name, int value01)
{
    if (value01 == 0) {
        motors_driver_stop_all();
        return;
    }

    const int spd = clamp_i(s_speed_pct, 0, 100);

    if (strcmp(ds_name, DS_M_FWD) == 0) {
        motors_driver_move_forward(spd);
    } else if (strcmp(ds_name, DS_M_BACK) == 0) {
        motors_driver_move_backward(spd);
    } else if (strcmp(ds_name, DS_M_LEFT) == 0) {
        motors_driver_turn_left(spd);
    } else if (strcmp(ds_name, DS_M_RIGHT) == 0) {
        motors_driver_turn_right(spd);
    } else if (strcmp(ds_name, DS_M_STOP) == 0) {
        motors_driver_stop_all();
    } else if (strcmp(ds_name, DS_M_ROT_L) == 0) {
        motors_driver_turn_left(spd);
    } else if (strcmp(ds_name, DS_M_ROT_R) == 0) {
        motors_driver_turn_right(spd);
    } else if (strcmp(ds_name, DS_M_BRAKE) == 0) {
        motors_driver_stop_all();
    }
}

static void publish_batch(void)
{
    if (!s_connected || !s_client) return;

    float t = 0.0f, h = 0.0f;
    uint32_t age_ms = 0;
    const bool ok = temp_driver_get_last_ok(&t, &h, &age_ms);

    const int ldr_raw = sensors_get_ldr_raw();
    const int ldr_thr = sensors_get_ldr_threshold();

    const float setc = temp_driver_get_setpoint_c();
    const int   dlt  = sensors_get_ldr_delta();

    const int fan = temp_driver_is_fan_on() ? 1 : 0;
    const int led = sensors_is_ldr_led_on() ? 1 : 0;

    char payload[256];
    if (ok) {
        snprintf(payload, sizeof(payload),
                 "{"
                 "\"%s\":%.2f,"
                 "\"%s\":%.2f,"
                 "\"%s\":%d,"
                 "\"%s\":%d,"
                 "\"%s\":%.2f,"
                 "\"%s\":%d,"
                 "\"%s\":%d,"
                 "\"%s\":%d"
                 "}",
                 DS_TEMPERATURE, t,
                 DS_HUMIDITY,    h,
                 DS_LDR_RAW,     ldr_raw,
                 DS_LDR_THR,     ldr_thr,
                 DS_TEMP_SETPOINT, setc,
                 DS_LDR_DELTA,     dlt,
                 DS_TEMP_ALARM,    fan,
                 DS_LDR_LED,       led);
    } else {
        snprintf(payload, sizeof(payload),
                 "{"
                 "\"%s\":%d,"
                 "\"%s\":%d,"
                 "\"%s\":%.2f,"
                 "\"%s\":%d,"
                 "\"%s\":%d,"
                 "\"%s\":%d"
                 "}",
                 DS_LDR_RAW,       ldr_raw,
                 DS_LDR_THR,       ldr_thr,
                 DS_TEMP_SETPOINT, setc,
                 DS_LDR_DELTA,     dlt,
                 DS_TEMP_ALARM,    fan,
                 DS_LDR_LED,       led);
    }

    if (esp_mqtt_client_publish(s_client, "batch_ds", payload, 0, 0, 0) < 0) {
        ESP_LOGW(TAG, "Falha ao publicar batch_ds");
    }else{
        ESP_LOGW(TAG, "Publicou batch_ds");
    }
}

static bool parse_number(const char *buf, float *out_f)
{
    if (!buf || !out_f) return false;
    char *end = NULL;
    const float v = strtof(buf, &end);
    if (end == buf) return false;
    *out_f = v;
    return true;
}

static void handle_downlink_ds(const char *topic, int topic_len, const char *data, int data_len)
{
    const char *prefix = "downlink/ds/";
    const int prefix_len = (int)strlen(prefix);
    if (topic_len <= prefix_len) return;
    if (strncmp(topic, prefix, (size_t)prefix_len) != 0) return;

    char ds[64];
    int name_len = topic_len - prefix_len;
    if (name_len <= 0) return;
    if (name_len >= (int)sizeof(ds)) name_len = (int)sizeof(ds) - 1;
    memcpy(ds, topic + prefix_len, (size_t)name_len);
    ds[name_len] = '\0';

    char val[64];
    int vlen = data_len;
    if (vlen < 0) vlen = 0;
    if (vlen >= (int)sizeof(val)) vlen = (int)sizeof(val) - 1;
    memcpy(val, data, (size_t)vlen);
    val[vlen] = '\0';

    if (strcmp(ds, DS_TEMP_SETPOINT) == 0) {
        float f;
        if (parse_number(val, &f)) {
            f = clamp_f(f, 0.0f, 80.0f);
            temp_driver_set_setpoint_c(f);
        }
        return;
    }

    if (strcmp(ds, DS_LDR_DELTA) == 0) {
        float f;
        if (parse_number(val, &f)) {
            sensors_set_ldr_delta((int)f);
        }
        return;
    }

    if (strcmp(ds, DS_MOTOR_SPEED) == 0) {
        float f;
        if (parse_number(val, &f)) {
            s_speed_pct = slider255_to_pct((int)f);
            ESP_LOGI(TAG, "Motor speed slider=%d -> %d%%", (int)f, s_speed_pct);
        }
        return;
    }

    if (strcmp(ds, DS_M_FWD) == 0 ||
        strcmp(ds, DS_M_BACK) == 0 ||
        strcmp(ds, DS_M_LEFT) == 0 ||
        strcmp(ds, DS_M_RIGHT) == 0 ||
        strcmp(ds, DS_M_STOP) == 0 ||
        strcmp(ds, DS_M_ROT_L) == 0 ||
        strcmp(ds, DS_M_ROT_R) == 0 ||
        strcmp(ds, DS_M_BRAKE) == 0) {

        float f;
        if (parse_number(val, &f)) {
            motors_apply(ds, (f != 0.0f) ? 1 : 0);
        }
        return;
    }

    ESP_LOGW(TAG, "Downlink ignorado: ds=%s val=%s", ds, val);
}

static void mqtt_event(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        ESP_LOGI(TAG, "MQTT conectado (Blynk)");
        esp_mqtt_client_subscribe(s_client, "downlink/ds/#", 0);

        esp_mqtt_client_publish(s_client, "get/ds/" DS_TEMP_SETPOINT, "", 0, 0, 0);
        esp_mqtt_client_publish(s_client, "get/ds/" DS_LDR_DELTA, "", 0, 0, 0);
        esp_mqtt_client_publish(s_client, "get/ds/" DS_MOTOR_SPEED, "", 0, 0, 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "MQTT desconectado");
        break;

    case MQTT_EVENT_DATA:
        handle_downlink_ds(e->topic, e->topic_len, e->data, e->data_len);
        break;

    default:
        break;
    }
}

static void blynk_task(void *pv)
{
    (void)pv;
    const TickType_t period = pdMS_TO_TICKS(500);

    ESP_LOGW(TAG, "Blinky task iniciado.");

    while (1) {
        publish_batch();
        vTaskDelay(period);
    }
}

void comm_blynk_init(void)
{
    if (strlen(BLYNK_DEVICE_TOKEN) == 0) {
        ESP_LOGE(TAG, "BLYNK_DEVICE_TOKEN vazio. Crie comm_blynk_secrets.h com o token.");
        return;
    }

    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = "mqtts://blynk.cloud:8883",
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach, /* <-- campo correto no seu IDF */
        .credentials.username = "device",
        .credentials.authentication.password = BLYNK_DEVICE_TOKEN,
        .session.keepalive = 45,
        .network.disable_auto_reconnect = false,
    };

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "esp_mqtt_client_init falhou");
        return;
    }

    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event, NULL);

    ESP_ERROR_CHECK(esp_mqtt_client_start(s_client));

    xTaskCreate(blynk_task, "blynk_task", 4096, NULL, 8, NULL);
    ESP_LOGW(TAG, "esp_mqtt_client_init iniciou");
}
