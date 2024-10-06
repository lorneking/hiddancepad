#include "driver/gpio.h"
#include "esp_log.h"

/*
Initialize GPIO pin
PIN = GPIO pin to be initialized
DIRECTION = renamed to avoid confusion with PULLMODE - otherwise known as mode, either GPIO_MODE_INPUT or GPIO_MODE_OUTPUT
PULLMODE = pull mode, either GPIO_PULLUP_ONLY or GPIO_PULLDOWN_ONLY
*/

void init_gpio(gpio_num_t PIN, gpio_mode_t DIRECTION, gpio_pull_mode_t PULLMODE) {
    
    ESP_LOGI("GPIO", "Initializing GPIO %d...", PIN);

    gpio_reset_pin(PIN);
    gpio_set_direction(PIN, DIRECTION);
    gpio_set_pull_mode(PIN, PULLMODE);

    ESP_LOGI("GPIO", "GPIO %d Initialized.", PIN);
}

void control_gpio(gpio_num_t PIN, bool LEVEL) {
    gpio_set_level(PIN, LEVEL);
}
