#include "driver/gpio.h"
#include "esp_log.h"
#include "pin_defs.h"
#include "inttypes.h"

static const char *TAG = "GPIO Control";

/*
Initialize GPIO pin
PIN = GPIO pin to be initialized
DIRECTION = renamed to avoid confusion with PULLMODE - otherwise known as mode, either GPIO_MODE_INPUT or GPIO_MODE_OUTPUT
PULLMODE = pull mode, either GPIO_PULLUP_ONLY or GPIO_PULLDOWN_ONLY
*/

gpio_num_t map_pad_to_led_gate(uint32_t pad_value) {
    switch (pad_value) {
        case 0x4: return LED_1_GATE;
        case 0x5: return LED_2_GATE;
        case 0x6: return LED_3_GATE;
        case 0x7: return LED_4_GATE;
        default: return GPIO_NUM_NC; // Return an invalid GPIO if no match
    }
}

void init_gpio(gpio_num_t PIN, gpio_mode_t DIRECTION, gpio_pull_mode_t PULLMODE) {
    
    ESP_LOGI(TAG, "Initializing GPIO %d...", PIN);

    gpio_reset_pin(PIN);
    gpio_set_direction(PIN, DIRECTION);
    gpio_set_pull_mode(PIN, PULLMODE);

    ESP_LOGI(TAG, "GPIO %d Initialized.", PIN);
}

void control_gpio(uint32_t PIN, bool LEVEL) {
    gpio_set_level(map_pad_to_led_gate(PIN), LEVEL);
}

void pad_led_on(uint32_t pad_value) {
    gpio_num_t gpio_pin = map_pad_to_led_gate(pad_value);

    if (gpio_pin != GPIO_NUM_NC) {  // Ensure the GPIO is valid
        gpio_set_level(gpio_pin, 1);  // Set the corresponding LED gate HIGH
        // ESP_LOGI(TAG, "LED ON: GPIO %d\n", gpio_pin);
    } else {
        ESP_LOGI(TAG, "Invalid pad value: 0x%"PRIu32, pad_value);
    }
}

void pad_led_off(uint32_t pad_value) {
    gpio_num_t gpio_pin = map_pad_to_led_gate(pad_value);

    if (gpio_pin != GPIO_NUM_NC) {  // Ensure the GPIO is valid
        gpio_set_level(gpio_pin, 0);  // Set the corresponding LED gate HIGH
        // ESP_LOGI(TAG, "LED OFF: GPIO %d\n", gpio_pin);
    } else {
        ESP_LOGI(TAG, "Invalid pad value: 0x%"PRIu32, pad_value);
    }
}
