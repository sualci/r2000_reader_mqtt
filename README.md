# ESP32-C3 + UHF RFID Reader + MQTT

Este repositorio contén o código para un **ESP32-C3** conectado a un **módulo lector UHF RFID R2000**.  
O dispositivo le etiquetas pasivas UHF e envía os datos a través do protocolo **MQTT**.

## 🚀 Configuración da ESP32-C3

1. Instala [Visual Studio Code](https://code.visualstudio.com/).  
2. Engade a extensión **Espressif IDF (ESP-IDF)**.  
3. Configura a ruta do SDK de **ESP-IDF**.  
4. Conecta a placa ESP32-C3 por **USB** e selecciona o porto serie correcto.  
5. Escolle o tipo de placa (exemplo: `ESP32-C3`).  
6. Compila, carga o firmware e abre o monitor serie executando:

   ```bash
   idf.py build
   idf.py -p PORT flash monitor
