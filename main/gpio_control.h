#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "driver/gpio.h"

void init_gpio(gpio_num_t PIN, gpio_mode_t DIRECTION, gpio_pull_mode_t PULLMODE);
void control_gpio(gpio_num_t PIN, bool LEVEL);
gpio_num_t map_pad_to_led_gate(uint32_t pad_value);
void pad_led_on(uint32_t pad_value);
void pad_led_off(uint32_t pad_value);

#endif // GPIO_CONTROL_H