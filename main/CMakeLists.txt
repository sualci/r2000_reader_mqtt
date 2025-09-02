idf_component_register(SRCS "main_old.c" "main.c" "r2000.c" "tagset.c" "mqtt.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver nvs_flash esp_netif esp_event esp_wifi mqtt
                    )
