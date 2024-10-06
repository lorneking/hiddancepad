idf_component_register(
    SRCS "main.c" "gpio.c" "hx711.c" "web_server.c" "wifi.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash esp_http_server esp_wifi esp_event
    )
