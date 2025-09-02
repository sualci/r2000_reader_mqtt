#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "tagset.h"

static const char *TAG = "TAGSET";

static tag_entry_t g_tags[MAX_TAGS];
static int g_tag_count = 0;
static SemaphoreHandle_t g_mutex = NULL;

// Comentario (ES): Inicializa el mutex de protección del set.
void tagset_init(void) {
    g_mutex = xSemaphoreCreateMutex();
}

// Comentario (ES): Limpia el conjunto de etiquetas únicas del ciclo.
void tagset_clear(void) {
    if (g_mutex) xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_tag_count = 0;
    memset(g_tags, 0, sizeof(g_tags));
    if (g_mutex) xSemaphoreGive(g_mutex);
}

// Comentario (ES): Añade EPC si no existe o actualiza su mejor RSSI si mejora.
void tagset_add_or_update(const uint8_t *epc, uint16_t epc_len, int8_t rssi) {
    if (!epc || epc_len == 0 || epc_len > sizeof(g_tags[0].epc)) return;
    if (g_mutex) xSemaphoreTake(g_mutex, portMAX_DELAY);
    for (int i = 0; i < g_tag_count; ++i) {
        if (g_tags[i].epc_len == epc_len && memcmp(g_tags[i].epc, epc, epc_len) == 0) {
            if (rssi > g_tags[i].best_rssi) g_tags[i].best_rssi = rssi;
            if (g_mutex) xSemaphoreGive(g_mutex);
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
    if (g_mutex) xSemaphoreGive(g_mutex);
}

int tagset_count(void) {
    if (g_mutex) xSemaphoreTake(g_mutex, portMAX_DELAY);
    int n = g_tag_count;
    if (g_mutex) xSemaphoreGive(g_mutex);
    return n;
}

const tag_entry_t* tagset_get(int idx) {
    if (idx < 0 || idx >= g_tag_count) return NULL;
    return &g_tags[idx];
}

// Comentario (ES): Imprime el set único actual para depuración.
void tagset_print_current(void) {
    if (g_mutex) xSemaphoreTake(g_mutex, portMAX_DELAY);
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
    if (g_mutex) xSemaphoreGive(g_mutex);
}
