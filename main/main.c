// main.c — ESP-IDF (ESP32-C3 + R2000 en UART GPIO20/21)
// Compila con:
//   idf.py set-target esp32c3
//   idf.py build flash monitor
 
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
 
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
 
// --- WIFI/MQTT ---
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"
 
// ===== Config UART / Pines =====
#define R2000_UART_NUM UART_NUM_0
#define R2000_BAUD 115200
#define R2000_TXD GPIO_NUM_20 // ESP32-C3 TX -> RX del R2000
#define R2000_RXD GPIO_NUM_21 // ESP32-C3 RX <- TX del R2000
#define R2000_RXBUF_SIZE 2048
 
// ===== Protocolo R2000 =====
#define COMM_HEAD 0xAA
#define COMM_TYPE_ANSWER 0x01
#define COMM_TYPE_INFORM 0x02
#define COMM_END 0xDD
#define CMD_READ_CARD 0x22
#define CMD_MULTI_INVENTORY 0x27
 
// Ráfaga
#define DEBUG_EACH_READ   0
#define MULTI_CNT         800       // nº de rondas internas
#define MULTI_WINDOW_MS   800       // ventana de escucha tras enviar 0x27
#define CYCLE_PERIOD_MS   5000      // un ciclo cada 5 s
 
static volatile uint32_t g_cycle_id = 0;
static volatile uint32_t g_cycle_rx_hits = 0;
 
static const char *TAG = "R2000";
 
// --- WIFI/MQTT: credenciales y broker ---
#define WIFI_SSID       "WifiClara"
#define WIFI_PASS       "wificlara11"
#define WIFI_MAX_RETRY  5
 
#define MQTT_BROKER_URI "mqtt://broker.hivemq.com"   // cámbialo si quieres
#define MQTT_TOPIC      "r2000/tags"                 // tópico de publicación
 
// ===== Utils =====
static inline uint16_t total_frame_len_from_payload(uint8_t lenH, uint8_t lenL)
{
    return (uint16_t)((((uint16_t)lenH << 8) | lenL) + 7);
}
static uint8_t sum_checksum(const uint8_t *p, size_t n)
{
    uint8_t s = 0;
    for (size_t i = 0; i < n; ++i) s = (uint8_t)(s + p[i]);
    return s;
}
 
static void print_read(int8_t rssi, uint16_t pc, const uint8_t *epc, uint16_t epc_len)
{
    printf("READ: RSSI=%d dBm, EPC=", (int)rssi);
    for (uint16_t i = 0; i < epc_len; ++i) printf("%02X", epc[i]);
    printf("\r\n");
}
 
// ===== Set ÚNICO POR CICLO =====
#define MAX_TAGS 128
typedef struct {
    uint8_t  epc[64];
    uint16_t epc_len;
    int8_t   best_rssi;
} tag_entry_t;
 
static tag_entry_t g_tags[MAX_TAGS];
static int g_tag_count = 0;
static SemaphoreHandle_t g_tags_mutex = NULL;
 
static void tags_clear(void)
{
    g_tag_count = 0;
    memset(g_tags, 0, sizeof(g_tags));
}
static void tags_add_or_update(const uint8_t *epc, uint16_t epc_len, int8_t rssi)
{
    if (!epc || epc_len == 0 || epc_len > sizeof(g_tags[0].epc)) return;
    for (int i = 0; i < g_tag_count; ++i) {
        if (g_tags[i].epc_len == epc_len && memcmp(g_tags[i].epc, epc, epc_len) == 0) {
            if (rssi > g_tags[i].best_rssi) g_tags[i].best_rssi = rssi;
            return;
        }
    }
    if (g_tag_count < MAX_TAGS) {
        tag_entry_t *e = &g_tags[g_tag_count++];
        e->epc_len = epc_len;
        memcpy(e->epc, epc, epc_len);
        e->best_rssi = rssi;
    } else {
        ESP_LOGW(TAG, "Set de únicas lleno (%d)", MAX_TAGS);
    }
}
static void tags_print_current(void)
{
    printf("\r\n=== TAGS ÚNICAS DEL CICLO (%d) ===\r\n", g_tag_count);
    if (g_tag_count == 0) {
        printf("(ninguna)\r\n");
    } else {
        for (int i = 0; i < g_tag_count; ++i) {
            printf("[%02d] EPC=", i);
            for (uint16_t j = 0; j < g_tags[i].epc_len; ++j) printf("%02X", g_tags[i].epc[j]);
            printf("  | best RSSI=%d dBm\r\n", (int)g_tags[i].best_rssi);
        }
    }
    printf("===================================\r\n");
}
 
// ===== Prototipos UART/RFID =====
static esp_err_t r2000_send_read_card(void);
static void r2000_handle_frame(const uint8_t *p, uint16_t len);
static void r2000_process_stream(const uint8_t *chunk, size_t chunk_len);
static void rx_task(void *arg);
static void inventory_task(void *arg);
static void r2000_uart_init(void);
 
// ===== Envío: READ_CARD (0x22) =====
static esp_err_t r2000_send_read_card(void)
{
    uint8_t frame[7] = { COMM_HEAD, 0x00, CMD_READ_CARD, 0x00, 0x00, 0x00, COMM_END };
    frame[5] = sum_checksum(&frame[1], 4);
    int wr = uart_write_bytes(R2000_UART_NUM, (const char *)frame, sizeof(frame));
    if (wr != (int)sizeof(frame)) {
        ESP_LOGW(TAG, "uart_write_bytes parcial (%d/%d)", wr, (int)sizeof(frame));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, ">> READ_CARD cmd enviado");
    return ESP_OK;
}
 
// ===== Parseo de un frame completo =====
static void r2000_handle_frame(const uint8_t *p, uint16_t len)
{
    if (len < 7) return;
 
    uint8_t type = p[1];
    uint8_t cmd  = p[2];
    uint8_t lenH = p[3];
    uint8_t lenL = p[4];
    uint16_t expected_total = total_frame_len_from_payload(lenH, lenL);
 
    if (expected_total != len) return;
    if (p[0] != COMM_HEAD || p[len - 1] != COMM_END) return;
 
    uint8_t calc = sum_checksum(&p[1], (size_t)len - 3);
    if (calc != p[len - 2]) return;
 
    const uint8_t *pl = &p[5];
    uint16_t pl_len = (uint16_t)((((uint16_t)lenH << 8) | lenL));
 
    // Notificación inventario: TYPE=0x02, CMD=0x22/0x27
    if (type == COMM_TYPE_INFORM && (cmd == CMD_READ_CARD || cmd == CMD_MULTI_INVENTORY)) {
        if (pl_len < 5) return; // RSSI(1) + PC(2) + CRC(2)
        int8_t   rssi    = (int8_t)pl[0];
        uint16_t pc      = (uint16_t)((pl[1] << 8) | pl[2]);
        uint16_t epc_len = (uint16_t)(pl_len - 5);
        const uint8_t *epc = &pl[3];
 
#if DEBUG_EACH_READ
        print_read(rssi, pc, epc, epc_len);
#endif
        if (g_tags_mutex) xSemaphoreTake(g_tags_mutex, portMAX_DELAY);
        tags_add_or_update(epc, epc_len, rssi);
        if (g_tags_mutex) xSemaphoreGive(g_tags_mutex);
        g_cycle_rx_hits++;
    }
}
 
// ===== Parseo sobre flujo (acumula y extrae frames) =====
static void r2000_process_stream(const uint8_t *chunk, size_t chunk_len)
{
    static uint8_t acc[4096];
    static size_t acc_len = 0;
 
    if (chunk_len == 0) return;
    if (acc_len + chunk_len > sizeof(acc)) {
        ESP_LOGW(TAG, "Overflow acumulador; reseteo");
        acc_len = 0;
    }
    memcpy(acc + acc_len, chunk, chunk_len);
    acc_len += chunk_len;
 
    size_t i = 0;
    while (acc_len - i >= 7) {
        if (acc[i] != COMM_HEAD) { i++; continue; }
        if (acc_len - i < 5) break;
        uint8_t lenH = acc[i + 3];
        uint8_t lenL = acc[i + 4];
        uint16_t total = total_frame_len_from_payload(lenH, lenL);
        if (total < 7) { i++; continue; }
        if (acc_len - i < total) break;
        if (acc[i + total - 1] != COMM_END) { i++; continue; }
        r2000_handle_frame(&acc[i], total);
        i += total;
    }
 
    if (i > 0) {
        size_t remaining = acc_len - i;
        memmove(acc, acc + i, remaining);
        acc_len = remaining;
    }
}
 
// ===== Tarea RX =====
static void rx_task(void *arg)
{
    uint8_t *buf = (uint8_t *)malloc(512);
    if (!buf) { ESP_LOGE(TAG, "malloc RX failed"); vTaskDelete(NULL); return; }
    while (1) {
        int n = uart_read_bytes(R2000_UART_NUM, buf, 512, pdMS_TO_TICKS(100));
        if (n > 0) r2000_process_stream(buf, (size_t)n);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
 
// 0x27 MULTI INVENTORY con contador de rondas
static esp_err_t r2000_send_multi_inventory_cnt(uint16_t cnt)
{
    uint8_t hdr[5] = {0xAA, 0x00, 0x27, 0x00, 0x03};
    uint8_t pl[3]  = {0x22, (uint8_t)(cnt >> 8), (uint8_t)cnt};
    uint8_t sum = hdr[1] + hdr[2] + hdr[3] + hdr[4] + pl[0] + pl[1] + pl[2];
    uint8_t tail[2] = {sum, 0xDD};
    uart_write_bytes(R2000_UART_NUM, (const char *)hdr, 5);
    uart_write_bytes(R2000_UART_NUM, (const char *)pl, 3);
    uart_write_bytes(R2000_UART_NUM, (const char *)tail, 2);
    ESP_LOGI(TAG, ">> MULTI cnt=%u", (unsigned)cnt);
    return ESP_OK;
}
static esp_err_t r2000_stop_multi(void)
{
    uint8_t frame[7] = {0xAA, 0x00, 0x28, 0x00, 0x00, 0x28, 0xDD};
    uart_write_bytes(R2000_UART_NUM, (const char *)frame, 7);
    ESP_LOGI(TAG, ">> STOP MULTI");
    return ESP_OK;
}
 
// --- WIFI/MQTT: helpers simples ---
// Wi-Fi
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
 
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
 
static void wifi_init_sta(void)
{
    // NVS
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
 
    ESP_LOGI("WIFI", "Conectando a \"%s\" ...", WIFI_SSID);
 
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdTRUE, pdFALSE, pdMS_TO_TICKS(15000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("WIFI", "¡Conectado y con IP!");
    } else {
        ESP_LOGE("WIFI", "No se logró conectar.");
    }
}
 
// MQTT
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool s_mqtt_connected = false;
 
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t eid, void *event_data)
{
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;
    if (eid == MQTT_EVENT_CONNECTED) {
        s_mqtt_connected = true;
        ESP_LOGI("MQTT", "Conectado");
    } else if (eid == MQTT_EVENT_DISCONNECTED) {
        s_mqtt_connected = false;
        ESP_LOGW("MQTT", "Desconectado");
    } else if (eid == MQTT_EVENT_ERROR) {
        ESP_LOGW("MQTT", "Error");
    }
}
static void mqtt_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt);
}
 
// Convierte EPC (bytes) a hex "E28069..."
static void epc_to_hex(const uint8_t *epc, uint16_t len, char *out, size_t outsz)
{
    size_t need = (size_t)len*2 + 1;
    if (!out || outsz < need) { if (out && outsz) out[0]=0; return; }
    for (uint16_t i=0;i<len;++i) sprintf(out + 2*i, "%02X", epc[i]);
    out[need-1] = 0;
}
 
// Publica la lista ÚNICA del ciclo: {"count":N,"tags":["EPC1","EPC2",...]}
static void mqtt_publish_current(void)
{
    if (!s_mqtt || !s_mqtt_connected) {
        ESP_LOGW("MQTT", "No conectado; no envío");
        return;
    }
 
    // Estima tamaño
    size_t cap = 32 + (size_t)g_tag_count * (2*64 + 4);
    char *json = (char*)malloc(cap);
    if (!json) { ESP_LOGE("MQTT", "malloc"); return; }
 
    int used = snprintf(json, cap, "{\"count\":%d,\"tags\":[", g_tag_count);
    if (used < 0) { free(json); return; }
 
    for (int i = 0; i < g_tag_count; ++i) {
        char epc_hex[129] = {0};
        epc_to_hex(g_tags[i].epc, g_tags[i].epc_len, epc_hex, sizeof(epc_hex));
        int n = snprintf(json + used, cap - used, "%s\"%s\"",
                         (i==0 ? "" : ","), epc_hex);
        if (n < 0) { free(json); return; }
        used += n;
    }
    used += snprintf(json + used, cap - used, "]}");
 
    esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC, json, used, 0, 0);
    ESP_LOGI("MQTT", "Publicado %d bytes en %s", used, MQTT_TOPIC);
    free(json);
}
// --- FIN WIFI/MQTT ---
 
// ===== Tarea inventario =====
static void inventory_task(void *arg)
{
    const TickType_t cycle  = pdMS_TO_TICKS(CYCLE_PERIOD_MS);
    const TickType_t window = pdMS_TO_TICKS(MULTI_WINDOW_MS);
    const TickType_t settle = pdMS_TO_TICKS(200); // dejar que RX drene
 
    while (1)
    {
        // 1) limpiar set del ciclo
        if (g_tags_mutex) { xSemaphoreTake(g_tags_mutex, portMAX_DELAY); tags_clear(); xSemaphoreGive(g_tags_mutex); }
        else { tags_clear(); }
        g_cycle_id++; g_cycle_rx_hits = 0;
 
        // 2) lanzar MULTI y escuchar
        r2000_send_multi_inventory_cnt(MULTI_CNT);
        vTaskDelay(window);
        r2000_stop_multi();
 
        // 3) drenar UART
        vTaskDelay(settle);
 
        // 4) imprimir y publicar
        if (g_tags_mutex) {
            xSemaphoreTake(g_tags_mutex, portMAX_DELAY);
            tags_print_current();
            mqtt_publish_current();          // <-- ENVÍO MQTT AQUÍ
            xSemaphoreGive(g_tags_mutex);
        } else {
            tags_print_current();
            mqtt_publish_current();
        }
 
        vTaskDelay(cycle);
    }
}
 
// ===== Init UART =====
static void r2000_uart_init(void)
{
    if (uart_is_driver_installed(R2000_UART_NUM)) {
        uart_driver_delete(R2000_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_ERROR_CHECK(uart_driver_install(R2000_UART_NUM, R2000_RXBUF_SIZE, 0, 0, NULL, 0));
 
    uart_config_t cfg = {0};
    cfg.baud_rate = R2000_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    ESP_ERROR_CHECK(uart_param_config(R2000_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(R2000_UART_NUM, R2000_TXD, R2000_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush_input(R2000_UART_NUM);
}
 
// ===== app_main =====
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 + R2000 (UART %d) @ %d baud", (int)R2000_UART_NUM, R2000_BAUD);
 
    g_tags_mutex = xSemaphoreCreateMutex();
    if (!g_tags_mutex) { ESP_LOGE(TAG, "No mutex"); return; }
 
    r2000_uart_init();
 
    // --- WIFI/MQTT ---
    wifi_init_sta();   // conecta a WifiClara y espera IP
    mqtt_start();      // arranca cliente MQTT
    // --- FIN WIFI/MQTT ---
 
    xTaskCreate(rx_task,        "r2000_rx",  4096, NULL, 10, NULL);
    xTaskCreate(inventory_task, "r2000_inv", 2048, NULL,  9, NULL);
}