#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "mqtt_client.h"

// Configuración Wi-Fi/MQTT (ajusta a tu entorno)
#define WIFI_SSID        "WifiClara"
#define WIFI_PASS        "*********"
#define WIFI_MAX_RETRY   5

#define MQTT_BROKER_URI  "mqtt://broker.hivemq.com"
#define MQTT_TOPIC       "r2000/tags"

// Estado MQTT
extern esp_mqtt_client_handle_t s_mqtt;
extern volatile bool s_mqtt_connected;

// Inicialización de red y MQTT
void wifi_init_sta(void);
void mqtt_start(void);

// Publicación del set actual como JSON {"count":N,"tags":[...]}
void mqtt_publish_current(void);

// Utilidad para convertir EPC a hex
void epc_to_hex(const uint8_t *epc, uint16_t len, char *out, size_t outsz);
