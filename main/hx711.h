#ifndef HX711_H
#define HX711_H

#include "driver/gpio.h"
#include "esp_err.h"
// #include "esp_http_server.h"

// Define constants for gain
#define GAIN_128 1
#define GAIN_64 3
#define GAIN_32 2

// Define constants for bit order
#define MSBFIRST 1
#define LSBFIRST 0

typedef struct {
    gpio_num_t PD_SCK;
    gpio_num_t DOUT;
    uint8_t GAIN;
    long OFFSET;
    float SCALE;
} HX711;

void hx711_init(HX711 *hx711, gpio_num_t dout, gpio_num_t pd_sck, uint8_t gain);
bool hx711_is_ready(HX711 *hx711);
void hx711_set_gain(HX711 *hx711, uint8_t gain);
long hx711_read(HX711 *hx711);
void hx711_wait_ready(HX711 *hx711, unsigned long delay_ms);
bool hx711_wait_ready_retry(HX711 *hx711, int retries, unsigned long delay_ms);
bool hx711_wait_ready_timeout(HX711 *hx711, unsigned long timeout, unsigned long delay_ms);
long hx711_read_average(HX711 *hx711, uint8_t times);
double hx711_get_value(HX711 *hx711, uint8_t times);
float hx711_get_units(HX711 *hx711, uint8_t times);
void hx711_tare(HX711 *hx711, uint8_t times);
void hx711_set_scale(HX711 *hx711, float scale);
float hx711_get_scale(HX711 *hx711);
void hx711_set_offset(HX711 *hx711, long offset);
long hx711_get_offset(HX711 *hx711);
void hx711_power_down(HX711 *hx711);
void hx711_power_up(HX711 *hx711);
uint8_t hx711_shift_in_slow(gpio_num_t dataPin, gpio_num_t clockPin, uint8_t bitOrder);

// esp_err_t hx711_get_handler(httpd_req_t *req);

#endif // HX711_H