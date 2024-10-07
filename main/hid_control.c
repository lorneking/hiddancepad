#include "stdio.h"
#include "class/hid/hid_device.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "hid_control.h"

static const char *TAG = "HID Control";

void send_hid_keypress(uint8_t keycode) {
    ESP_LOGI(TAG, "Sending Keyboard report 0x%02X", keycode);
    uint8_t keycode_buf[6] = {keycode};
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode_buf);
    vTaskDelay(pdMS_TO_TICKS(13));
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
}