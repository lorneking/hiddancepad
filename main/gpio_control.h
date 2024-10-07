#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "driver/gpio.h"

void init_gpio(gpio_num_t PIN, gpio_mode_t DIRECTION, gpio_pull_mode_t PULLMODE);
void control_gpio(gpio_num_t PIN, bool LEVEL);

#endif // GPIO_CONTROL_H