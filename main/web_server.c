#include "esp_http_server.h"
#include "esp_log.h"
#include "hx711.h"
#include "config.h"

HX711 scaleread1, scaleread2, scaleread3, scaleread4;

// HTTP Server Handlers
esp_err_t hello_get_handler(httpd_req_t *req) {
    const char* resp_str = "Welcome to the DDR Dance Pad Controller!";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}

#if PADS_USE_LOAD_CELLS
// TODO: Refactor this into a single function, pass scales to function
esp_err_t hx711_get_handler(httpd_req_t *req) {
    long sensor_value1 = hx711_read(&scaleread1);
    long sensor_value2 = hx711_read(&scaleread2);
    long sensor_value3 = hx711_read(&scaleread3);
    long sensor_value4 = hx711_read(&scaleread4);

    char resp_str[256];
    snprintf(resp_str, sizeof(resp_str), 
             "HX711 Sensor Values:\nSensor 1: %ld\nSensor 2: %ld\nSensor 3: %ld\nSensor 4: %ld", 
             sensor_value1, sensor_value2, sensor_value3, sensor_value4);
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}
#endif

void start_webserver(void) {
    ESP_LOGI("WEB", "Starting webserver...");
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_get = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = hello_get_handler,
        };
        httpd_register_uri_handler(server, &uri_get);

        #if PADS_USE_LOAD_CELLS
        httpd_uri_t uri_hx711 = {
            .uri      = "/hx711",
            .method   = HTTP_GET,
            .handler  = hx711_get_handler,
        };
        httpd_register_uri_handler(server, &uri_hx711);
        #endif
    }
}

