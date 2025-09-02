#pragma once
#include <stdint.h>
#include "esp_err.h"

// --- UART / Pines (ajusta si hace falta) ---
#define R2000_UART_NUM      UART_NUM_0
#define R2000_BAUD          115200
#define R2000_TXD           GPIO_NUM_20 // ESP32-C3 TX -> RX del R2000
#define R2000_RXD           GPIO_NUM_21 // ESP32-C3 RX <- TX del R2000
#define R2000_RXBUF_SIZE    2048

// --- Protocolo R2000 ---
#define COMM_HEAD           0xAA
#define COMM_TYPE_ANSWER    0x01
#define COMM_TYPE_INFORM    0x02
#define COMM_END            0xDD
#define CMD_READ_CARD       0x22
#define CMD_MULTI_INVENTORY 0x27

// Parámetros de ráfaga / ciclo
#define MULTI_CNT        800        // rondas internas del inventario múltiple
#define MULTI_WINDOW_MS  800        // ventana de escucha tras 0x27
#define CYCLE_PERIOD_MS  5000       // un ciclo completo cada 5 s

// Estado de ciclo (sólo lectura desde fuera)
extern volatile uint32_t g_cycle_id;
extern volatile uint32_t g_cycle_rx_hits;

// Inicialización de UART y tareas relacionadas
void r2000_uart_init(void);
void r2000_start_tasks(void *unused);

// Envío de comandos
esp_err_t r2000_send_read_card(void);
esp_err_t r2000_send_multi_inventory_cnt(uint16_t cnt);
esp_err_t r2000_stop_multi(void);

// Inyección de chunk recibido (para tests)
void r2000_process_stream(const uint8_t *chunk, size_t chunk_len);
