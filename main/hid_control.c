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
    // vTaskDelay(pdMS_TO_TICKS(13));
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
}

void start_hid_keypress (uint8_t keycode) {
    ESP_LOGI(TAG, "Starting Keyboard report 0x%02X", keycode);
    uint8_t keycode_buf[6] = {keycode};
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode_buf);
}

void end_hid_keypress (uint8_t keycode) {
    ESP_LOGI(TAG, "Ending Keyboard report 0x%02X", keycode);
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
}

uint8_t pad_to_keycode(uint32_t pad_value) {
    switch (pad_value) {
        case 0x4: return 0x52;
        case 0x5: return 0x50;
        case 0x6: return 0x4f;
        case 0x7: return 0x51;
        default: return 0x00; // Return an invalid keypress if no match
    }
}