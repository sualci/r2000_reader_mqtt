#pragma once
#include <stdint.h>

// Tamaño máximo por ciclo
#define MAX_TAGS 128

typedef struct {
    uint8_t  epc[64];
    uint16_t epc_len;
    int8_t   best_rssi;
} tag_entry_t;

// API del set de etiquetas únicas del ciclo
void tagset_init(void);                 // crea el mutex interno
void tagset_clear(void);                // limpia set (thread-safe)
void tagset_add_or_update(const uint8_t *epc, uint16_t epc_len, int8_t rssi);
int  tagset_count(void);                // número de etiquetas
const tag_entry_t* tagset_get(int idx); // acceso de solo lectura
void tagset_print_current(void);        // debug por consola
