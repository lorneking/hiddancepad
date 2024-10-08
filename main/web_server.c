#include "esp_http_server.h"
#include "esp_log.h"
#include "hx711.h"
#include "config.h"
#include "driver/gpio.h"
#include "pin_defs.h"
#include "config.h"
#include "touchpad.h"

const char* html_page = "<!DOCTYPE html>"
"<html>"
"<head>"
"<title>DDR Dance Pad Controller</title>"
"<script>"
"var ws;"
"function startWebSocket() {"
"  ws = new WebSocket('ws://' + window.location.hostname + '/ws');"
"  ws.onopen = function() {"
"    console.log('WebSocket connection opened.');"
"  };"
"  ws.onmessage = function(event) {"
"    var data = JSON.parse(event.data);"
"    document.getElementById('down').innerHTML = 'DOWN: ' + data.sensors[0];"
"    document.getElementById('up').innerHTML = 'UP: ' + data.sensors[1];"
"    document.getElementById('left').innerHTML = 'LEFT: ' + data.sensors[2];"
"    document.getElementById('right').innerHTML = 'RIGHT: ' + data.sensors[3];"
"    document.getElementById('down-led').innerHTML = 'DOWN LED: ' + (data.leds[0] ? 'ON' : 'OFF');"
"    document.getElementById('up-led').innerHTML = 'UP LED: ' + (data.leds[1] ? 'ON' : 'OFF');"
"    document.getElementById('left-led').innerHTML = 'LEFT LED: ' + (data.leds[2] ? 'ON' : 'OFF');"
"    document.getElementById('right-led').innerHTML = 'RIGHT LED: ' + (data.leds[3] ? 'ON' : 'OFF');"
"  };"
"  ws.onclose = function() {"
"    console.log('WebSocket connection closed.');"
"  };"
"}"
"window.onload = startWebSocket;"
"</script>"
"</head>"
"<body>"
"<h1>DDR Dance Pad Controller</h1>"
"<p id='down'>DOWN: Loading...</p>"
"<p id='up'>UP: Loading...</p>"
"<p id='left'>LEFT: Loading...</p>"
"<p id='right'>RIGHT: Loading...</p>"
"<h2>LEDs</h2>"
"<p id='down-led'>DOWN LED: Loading...</p>"
"<p id='up-led'>UP LED: Loading...</p>"
"<p id='left-led'>LEFT LED: Loading...</p>"
"<p id='right-led'>RIGHT LED: Loading...</p>"
"</body>"
"</html>";

int led_state1 = 0, led_state2 = 0, led_state3 = 0, led_state4 = 0;
long sensor_value1 = 0, sensor_value2 = 0, sensor_value3 = 0, sensor_value4 = 0;

esp_err_t hello_get_handler(httpd_req_t *req) {
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

static void update_sensor_states() {
    sensor_value1 = tp_read(DOWN_ARROW_TOUCH);
    sensor_value2 = tp_read(UP_ARROW_TOUCH);
    sensor_value3 = tp_read(LEFT_ARROW_TOUCH);
    sensor_value4 = tp_read(RIGHT_ARROW_TOUCH);
}

static void update_led_states() {
    led_state1 = gpio_get_level(LED_1_GATE);
    led_state2 = gpio_get_level(LED_2_GATE);
    led_state3 = gpio_get_level(LED_3_GATE);
    led_state4 = gpio_get_level(LED_4_GATE);
}

// WebSocket handler to send real-time data
esp_err_t websocket_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;

        // Create JSON payload with sensor, button, and LED values
        update_sensor_states();
        update_led_states();

        char payload[256];
        snprintf(payload, sizeof(payload), 
                 "{\"sensors\": [%ld, %ld, %ld, %ld], \"leds\": [%d, %d, %d, %d]}",
                 sensor_value1, sensor_value2, sensor_value3, sensor_value4,
                 led_state1, led_state2, led_state3, led_state4);

        // Prepare the WebSocket frame with the payload
        ws_pkt.payload = (uint8_t*)payload;
        ws_pkt.len = strlen(payload);

        // Send the WebSocket frame
        return httpd_ws_send_frame(req, &ws_pkt);
    }

    return ESP_OK;
}

// Start WebSocket server
void start_websocket_server(httpd_handle_t server) {
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = NULL,
        .is_websocket = true  // Enable WebSocket for this URI
    };
    httpd_register_uri_handler(server, &ws_uri);
}

// Start HTTP and WebSocket server
void start_webserver(void) {
    ESP_LOGI("WEB", "Starting webserver...");
    
    // Create an HTTP server config
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    // Initialize server handle
    httpd_handle_t server = NULL;

    // Start the HTTP server
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers here
        httpd_uri_t uri_get = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = hello_get_handler,
        };
        httpd_register_uri_handler(server, &uri_get);

        // Start WebSocket server
        start_websocket_server(server);  // Pass server handle to WebSocket
    } else {
        ESP_LOGE("WEB", "Failed to start webserver");
    }
}

#if PADS_USE_LOAD_CELLS

HX711 scaleread1, scaleread2, scaleread3, scaleread4;

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

