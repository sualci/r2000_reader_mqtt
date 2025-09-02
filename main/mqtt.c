#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "mqtt.h"
#include "tagset.h"

static const char *TAGW = "WIFI";
static const char *TAGM = "MQTT";

// --- Wi-Fi (STA) ---
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) { esp_wifi_connect(); s_retry_num++; }
        else { xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0; xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Comentario (ES): Inicializa NVS, pila de red, Wi-Fi STA y espera IP o timeout.
void wifi_init_sta(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wicfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wicfg));

    wifi_config_t cfg = {0};
    strncpy((char*)cfg.sta.ssid, WIFI_SSID, sizeof(cfg.sta.ssid));
    strncpy((char*)cfg.sta.password, WIFI_PASS, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAGW, "Conectando a \"%s\" ...", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdTRUE, pdFALSE, pdMS_TO_TICKS(15000));
    if (bits & WIFI_CONNECTED_BIT) ESP_LOGI(TAGW, "¡Conectado y con IP!");
    else                           ESP_LOGE(TAGW, "No se logró conectar.");
}

// --- MQTT ---
esp_mqtt_client_handle_t s_mqtt = NULL;
volatile bool s_mqtt_connected = false;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t eid, void *event_data) {
    if (eid == MQTT_EVENT_CONNECTED)      { s_mqtt_connected = true;  ESP_LOGI(TAGM, "Conectado"); }
    else if (eid == MQTT_EVENT_DISCONNECTED){ s_mqtt_connected = false; ESP_LOGW(TAGM, "Desconectado"); }
    else if (eid == MQTT_EVENT_ERROR)     { ESP_LOGW(TAGM, "Error"); }
}

void mqtt_start(void) {
    esp_mqtt_client_config_t cfg = { .broker.address.uri = MQTT_BROKER_URI };
    s_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt);
}

// --- Utilidad: EPC bytes -> string hex "E28069..." ---
void epc_to_hex(const uint8_t *epc, uint16_t len, char *out, size_t outsz) {
    size_t need = (size_t)len * 2 + 1;
    if (!out || outsz < need) { if (out && outsz) out[0] = 0; return; }
    for (uint16_t i = 0; i < len; ++i) sprintf(out + 2*i, "%02X", epc[i]);
    out[need - 1] = 0;
}

// --- Publicación JSON del set actual ---
// Comentario (ES): Construye {"count":N,"tags":["..."]} y publica en MQTT_TOPIC.
void mqtt_publish_current(void) {
    if (!s_mqtt || !s_mqtt_connected) { ESP_LOGW(TAGM, "No conectado; no envío"); return; }

    int n = tagset_count();
    // Estimación grosera de capacidad: 32 + N * (2*64 + 4)
    size_t cap = 32 + (size_t)n * (2*64 + 4);
    char *json = (char*)malloc(cap);
    if (!json) { ESP_LOGE(TAGM, "malloc"); return; }

    int used = snprintf(json, cap, "{\"count\":%d,\"tags\":[", n);
    if (used < 0) { free(json); return; }

    for (int i = 0; i < n; ++i) {
        const tag_entry_t *e = tagset_get(i);
        if (!e) continue;
        char epc_hex[129] = {0};
        epc_to_hex(e->epc, e->epc_len, epc_hex, sizeof(epc_hex));
        int m = snprintf(json + used, cap - used, "%s\"%s\"", (i==0 ? "" : ","), epc_hex);
        if (m < 0) { free(json); return; }
        used += m;
    }
    used += snprintf(json + used, cap - used, "]}");

    esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC, json, used, 0, 0);
    ESP_LOGI(TAGM, "Publicado %d bytes en %s", used, MQTT_TOPIC);
    free(json);
}
