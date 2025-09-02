#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "r2000.h"
#include "tagset.h"
#include "mqtt.h"

static const char *TAG = "R2000";

volatile uint32_t g_cycle_id = 0;
volatile uint32_t g_cycle_rx_hits = 0;

// --- Utilidades internas ---
static inline uint16_t total_frame_len_from_payload(uint8_t lenH, uint8_t lenL) {
    return (uint16_t)((((uint16_t)lenH << 8) | lenL) + 7);
}
static uint8_t sum_checksum(const uint8_t *p, size_t n) {
    uint8_t s = 0; for (size_t i = 0; i < n; ++i) s = (uint8_t)(s + p[i]); return s;
}

// --- Envío: READ_CARD (0x22) ---
// Comentario (ES): Construye y envía el frame 0x22 para pedir una lectura única.
// El checksum es la suma de bytes [1..len-3] modulo 256.
esp_err_t r2000_send_read_card(void) {
    uint8_t frame[7] = {COMM_HEAD, 0x00, CMD_READ_CARD, 0x00, 0x00, 0x00, COMM_END};
    frame[5] = sum_checksum(&frame[1], 4);
    int wr = uart_write_bytes(R2000_UART_NUM, (const char *)frame, sizeof(frame));
    if (wr != (int)sizeof(frame)) {
        ESP_LOGW(TAG, "uart_write_bytes parcial (%d/%d)", wr, (int)sizeof(frame));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, ">> READ_CARD enviado");
    return ESP_OK;
}

// --- Envío: MULTI INVENTORY con N rondas (0x27) ---
// Comentario (ES): Lanza inventario múltiple interno en el lector (anticolisión).
esp_err_t r2000_send_multi_inventory_cnt(uint16_t cnt) {
    uint8_t hdr[5] = {0xAA, 0x00, 0x27, 0x00, 0x03};
    uint8_t pl[3]  = {0x22, (uint8_t)(cnt >> 8), (uint8_t)cnt}; // 0x22 = sesión/param
    uint8_t sum = hdr[1] + hdr[2] + hdr[3] + hdr[4] + pl[0] + pl[1] + pl[2];
    uint8_t tail[2] = {sum, 0xDD};
    uart_write_bytes(R2000_UART_NUM, (const char*)hdr, 5);
    uart_write_bytes(R2000_UART_NUM, (const char*)pl, 3);
    uart_write_bytes(R2000_UART_NUM, (const char*)tail, 2);
    ESP_LOGI(TAG, ">> MULTI cnt=%u", (unsigned)cnt);
    return ESP_OK;
}

esp_err_t r2000_stop_multi(void) {
    uint8_t frame[7] = {0xAA, 0x00, 0x28, 0x00, 0x00, 0x28, 0xDD};
    uart_write_bytes(R2000_UART_NUM, (const char*)frame, 7);
    ESP_LOGI(TAG, ">> STOP MULTI");
    return ESP_OK;
}

// --- Parseo de un frame completo ---
// Comentario (ES): Valida cabecera, longitud, checksum y fin; si es notificación
// de inventario (0x22/0x27), extrae RSSI, PC y EPC y actualiza el set único.
static void r2000_handle_frame(const uint8_t *p, uint16_t len) {
    if (len < 7) return;
    uint8_t type = p[1], cmd = p[2], lenH = p[3], lenL = p[4];
    uint16_t expected_total = total_frame_len_from_payload(lenH, lenL);
    if (expected_total != len) return;
    if (p[0] != COMM_HEAD || p[len - 1] != COMM_END) return;
    uint8_t calc = sum_checksum(&p[1], (size_t)len - 3);
    if (calc != p[len - 2]) return;

    const uint8_t *pl = &p[5];
    uint16_t pl_len = (uint16_t)((((uint16_t)lenH << 8) | lenL));

    if (type == COMM_TYPE_INFORM && (cmd == CMD_READ_CARD || cmd == CMD_MULTI_INVENTORY)) {
        if (pl_len < 5) return; // RSSI(1)+PC(2)+CRC(2)
        int8_t rssi = (int8_t)pl[0];
        // uint16_t pc = (uint16_t)((pl[1] << 8) | pl[2]); // No se usa ahora
        uint16_t epc_len = (uint16_t)(pl_len - 5);
        const uint8_t *epc = &pl[3];

        tagset_add_or_update(epc, epc_len, rssi);
        g_cycle_rx_hits++;
    }
}

// --- Parseo sobre flujo (acumulador) ---
// Comentario (ES): Ensambla frames a partir de chunks arbitrarios de UART.
void r2000_process_stream(const uint8_t *chunk, size_t chunk_len) {
    static uint8_t acc[4096];
    static size_t acc_len = 0;
    if (chunk_len == 0) return;
    if (acc_len + chunk_len > sizeof(acc)) { ESP_LOGW(TAG, "Overflow; reseteo"); acc_len = 0; }
    memcpy(acc + acc_len, chunk, chunk_len); acc_len += chunk_len;

    size_t i = 0;
    while (acc_len - i >= 7) {
        if (acc[i] != COMM_HEAD) { i++; continue; }
        if (acc_len - i < 5) break;
        uint8_t lenH = acc[i+3], lenL = acc[i+4];
        uint16_t total = total_frame_len_from_payload(lenH, lenL);
        if (total < 7) { i++; continue; }
        if (acc_len - i < total) break;
        if (acc[i + total - 1] != COMM_END) { i++; continue; }
        r2000_handle_frame(&acc[i], total);
        i += total;
    }
    if (i > 0) { size_t remaining = acc_len - i; memmove(acc, acc + i, remaining); acc_len = remaining; }
}

// --- Tarea RX UART ---
// Comentario (ES): Lee bytes de UART y alimenta el ensamblador de frames.
static void rx_task(void *arg) {
    uint8_t *buf = (uint8_t*)malloc(512);
    if (!buf) { ESP_LOGE(TAG, "malloc RX failed"); vTaskDelete(NULL); return; }
    while (1) {
        int n = uart_read_bytes(R2000_UART_NUM, buf, 512, pdMS_TO_TICKS(100));
        if (n > 0) r2000_process_stream(buf, (size_t)n);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- Tarea de inventario por ciclos ---
// Comentario (ES): Limpia set, lanza 0x27 con N rondas, espera ventana,
// drena UART, imprime/publica y duerme hasta el siguiente ciclo.
static void inventory_task(void *arg) {
    const TickType_t cycle  = pdMS_TO_TICKS(CYCLE_PERIOD_MS);
    const TickType_t window = pdMS_TO_TICKS(MULTI_WINDOW_MS);
    const TickType_t settle = pdMS_TO_TICKS(200);

    while (1) {
        tagset_clear();
        g_cycle_id++; g_cycle_rx_hits = 0;

        r2000_send_multi_inventory_cnt(MULTI_CNT);
        vTaskDelay(window);
        r2000_stop_multi();
        vTaskDelay(settle);

        tagset_print_current();      // salida por consola
        mqtt_publish_current();      // publicación JSON a broker

        vTaskDelay(cycle);
    }
}

// --- Init UART y arranque de tareas ---
void r2000_uart_init(void) {
    if (uart_is_driver_installed(R2000_UART_NUM)) {
        uart_driver_delete(R2000_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_ERROR_CHECK(uart_driver_install(R2000_UART_NUM, R2000_RXBUF_SIZE, 0, 0, NULL, 0));

    uart_config_t cfg = {
        .baud_rate = R2000_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(R2000_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(R2000_UART_NUM, R2000_TXD, R2000_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush_input(R2000_UART_NUM);
}

void r2000_start_tasks(void *unused) {
    xTaskCreate(rx_task,        "r2000_rx",  4096, NULL, 10, NULL);
    xTaskCreate(inventory_task, "r2000_inv", 2048, NULL,  9, NULL);
}
