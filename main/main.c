// main.c — ESP-IDF (ESP32-C3 + R2000)
// Compila con:
//   idf.py set-target esp32c3
//   idf.py build flash monitor

#include "esp_log.h"
#include "r2000.h"
#include "tagset.h"
#include "mqtt.h"

void app_main(void) {
    ESP_LOGI("APP", "ESP32-C3 + R2000 @ %d baud", R2000_BAUD);

    tagset_init();        // set único con mutex interno
    r2000_uart_init();    // UART para el R2000

    wifi_init_sta();      // conecta Wi-Fi y espera IP
    mqtt_start();         // arranca cliente MQTT

    r2000_start_tasks(NULL); // RX + inventario (ciclos)
}
