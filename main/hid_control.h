#ifndef HID_CONTROL_H
#define HID_CONTROL_H

#include "class/hid/hid_device.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

void send_hid_keypress(uint8_t keycode);

#endif // HID_CONTROL_H