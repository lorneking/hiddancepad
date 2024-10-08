#ifndef HID_CONTROL_H
#define HID_CONTROL_H

#include "stdio.h"
#include "class/hid/hid_device.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "hid_control.h"

void send_hid_keypress(uint8_t keycode);
void start_hid_keypress (uint8_t keycode);
void end_hid_keypress (uint8_t keycode);
uint8_t pad_to_keycode(uint32_t pad_value);

#endif // HID_CONTROL_H