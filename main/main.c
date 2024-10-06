#include <stdlib.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "gpio.h"
#include "hx711.h"
#include "wifi.h"
#include "web_server.h"
#include "esp_event.h"
#include "esp_task_wdt.h"

#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *TAG = "MAIN";

// Define GPIO Pins
#define HX711_SCK GPIO_NUM_8 // Common clock pin for all HX711s
#define HX711_1_DT GPIO_NUM_4 // Data pin for HX711 1
#define HX711_2_DT GPIO_NUM_5 // Data pin for HX711 2
#define HX711_3_DT GPIO_NUM_6 // Data pin for HX711 3
#define HX711_4_DT GPIO_NUM_7 // Data pin for HX711 4
#define LED_1_GATE GPIO_NUM_35 // Gate for LED 1
#define LED_2_GATE GPIO_NUM_36 // Gate for LED 2
#define LED_3_GATE GPIO_NUM_37 // Gate for LED 3
#define LED_4_GATE GPIO_NUM_38 // Gate for LED 4

// Define threshold values
long threshold = 10000; // Delta threshold for pad step detection
//int soundThreshold = 300; TODO: Implement sound threshold for sound-activated LEDs

HX711 scale1, scale2, scale3, scale4;
long prevWeight1, prevWeight2, prevWeight3, prevWeight4;

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
};

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}

/********* Application ***************/

typedef enum {
    MOUSE_DIR_RIGHT,
    MOUSE_DIR_DOWN,
    MOUSE_DIR_LEFT,
    MOUSE_DIR_UP,
    MOUSE_DIR_MAX,
} mouse_dir_t;

#define DISTANCE_MAX        125
#define DELTA_SCALAR        5

static void mouse_draw_square_next_delta(int8_t *delta_x_ret, int8_t *delta_y_ret)
{
    static mouse_dir_t cur_dir = MOUSE_DIR_RIGHT;
    static uint32_t distance = 0;

    // Calculate next delta
    if (cur_dir == MOUSE_DIR_RIGHT) {
        *delta_x_ret = DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_DOWN) {
        *delta_x_ret = 0;
        *delta_y_ret = DELTA_SCALAR;
    } else if (cur_dir == MOUSE_DIR_LEFT) {
        *delta_x_ret = -DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_UP) {
        *delta_x_ret = 0;
        *delta_y_ret = -DELTA_SCALAR;
    }

    // Update cumulative distance for current direction
    distance += DELTA_SCALAR;
    // Check if we need to change direction
    if (distance >= DISTANCE_MAX) {
        distance = 0;
        cur_dir++;
        if (cur_dir == MOUSE_DIR_MAX) {
            cur_dir = 0;
        }
    }
}

static void app_send_hid_keypress(uint8_t keycode)
{
    ESP_LOGI(TAG, "Sending Keyboard report");
    uint8_t keycode_buf[6] = {keycode};
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode_buf);
    vTaskDelay(pdMS_TO_TICKS(50));
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
}

static void app_send_mouse_move(int8_t delta_x, int8_t delta_y)
{
    ESP_LOGI(TAG, "Sending Mouse report");
    tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, delta_x, delta_y, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
}

void hx711_task(void *pvParameter) {

    /* Initialization of HX711 load cell amplifiers 
    Uncomment the appropriate section to adjust the sensitivity of the HX711 
    128 = 128x amplification - smallest measurement range, most noise, most sensitive 
    64 = 64x amplification - medium measurement range, medium noise, medium sensitivity
    32 = 32x amplification - widest measurement range, least noise, lowest sensitivity */

    /* Begin 128x amplification */
    // hx711_init(&scale1, HX711_1_DT, HX711_SCK, 128);
    // hx711_init(&scale2, HX711_2_DT, HX711_SCK, 128);
    // hx711_init(&scale3, HX711_3_DT, HX711_SCK, 128);
    // hx711_init(&scale4, HX711_4_DT, HX711_SCK, 128);
    /* End 128x amplification */

    /* Begin 64x amplification */
    hx711_init(&scale1, HX711_1_DT, HX711_SCK, 64);
    hx711_init(&scale2, HX711_2_DT, HX711_SCK, 64);
    hx711_init(&scale3, HX711_3_DT, HX711_SCK, 64);
    hx711_init(&scale4, HX711_4_DT, HX711_SCK, 64);
    /* End 64x amplification */

    /* Begin 32x amplification */
    // hx711_init(&scale1, HX711_1_DT, HX711_SCK, 32);
    // hx711_init(&scale2, HX711_2_DT, HX711_SCK, 32);
    // hx711_init(&scale3, HX711_3_DT, HX711_SCK, 32);
    // hx711_init(&scale4, HX711_4_DT, HX711_SCK, 32);
    /* End 32x amplification */

    static bool send_hid_data = false;

    while (1) {      

        long weight1 = hx711_read(&scale1);
        long weight2 = hx711_read(&scale2);
        long weight3 = hx711_read(&scale3);
        long weight4 = hx711_read(&scale4);

        if (tud_mounted()) {
            send_hid_data = true;
        }

        if (weight1 != -1) { 
            if ((weight1 - threshold) > prevWeight1) {    
                gpio_set_level(LED_1_GATE, 1);
                ESP_LOGI(TAG, "Pad 1 step detected");
                if (send_hid_data) {
                    app_send_hid_keypress(HID_KEY_ARROW_UP);
                }
            } else {  
                gpio_set_level(LED_1_GATE, 0);
            }
            prevWeight1 = weight1;
        }
        
        if (weight2 != -1) { 
            if ((weight2 - threshold) > prevWeight2) {    
                gpio_set_level(LED_2_GATE, 1);
                ESP_LOGI(TAG, "Pad 2 step detected");
                if (send_hid_data) {
                    app_send_hid_keypress(HID_KEY_ARROW_DOWN);
                }  
            } else {   
                gpio_set_level(LED_2_GATE, 0);
            }
            prevWeight2 = weight2;
        }

        if (weight3 != -1) { 
            if ((weight3 - threshold) > prevWeight3) {
                gpio_set_level(LED_3_GATE, 1);
                ESP_LOGI(TAG, "Pad 3 step detected");
                if (send_hid_data) {
                    app_send_hid_keypress(HID_KEY_ARROW_LEFT);
                }   
            } else {   
                gpio_set_level(LED_3_GATE, 0);
            }
            prevWeight3 = weight3;
        }

        if (weight4 != -1) { 
            if ((weight4 - threshold) > prevWeight4) {   
                gpio_set_level(LED_4_GATE, 1);
                ESP_LOGI(TAG, "Pad 4 step detected");
                if (send_hid_data) {
                    app_send_hid_keypress(HID_KEY_ARROW_RIGHT);
                } 
            } else { 
                gpio_set_level(LED_4_GATE, 0);
            }
            prevWeight4 = weight4;
        }

        vTaskDelay(pdMS_TO_TICKS(12)); // Delay for 12ms = 80Hz sample rate from HX711

        send_hid_data = ((weight1 - threshold) > prevWeight1) || ((weight2 - threshold) > prevWeight2) || ((weight3 - threshold) > prevWeight3) || ((weight4 - threshold) > prevWeight4);

    }
    
}

void app_main(void)
{    
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    start_webserver();

    // Initialize button that will trigger HID reports
    // const gpio_config_t boot_button_config = {
    //     .pin_bit_mask = BIT64(APP_BUTTON),
    //     .mode = GPIO_MODE_INPUT,
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .pull_up_en = true,
    //     .pull_down_en = false,
    // };
    // ESP_ERROR_CHECK(gpio_config(&boot_button_config));

    // USB Initialization
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
            .device_descriptor = NULL,
            .string_descriptor = hid_string_descriptor,
            .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
            .external_phy = false,
        #if (TUD_OPT_HIGH_SPEED)
                .fs_configuration_descriptor = hid_configuration_descriptor, // HID configuration descriptor for full-speed and high-speed are the same
                .hs_configuration_descriptor = hid_configuration_descriptor,
                .qualifier_descriptor = NULL,
        #else
                .configuration_descriptor = hid_configuration_descriptor,
        #endif // TUD_OPT_HIGH_SPEED
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    // Initialize LED driver GPIOs
    init_gpio(LED_1_GATE, GPIO_MODE_OUTPUT, GPIO_PULLUP_ONLY);
    init_gpio(LED_2_GATE, GPIO_MODE_OUTPUT, GPIO_PULLUP_ONLY);
    init_gpio(LED_3_GATE, GPIO_MODE_OUTPUT, GPIO_PULLUP_ONLY);
    init_gpio(LED_4_GATE, GPIO_MODE_OUTPUT, GPIO_PULLUP_ONLY);

    esp_log_level_set("wifi", ESP_LOG_VERBOSE);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    // Initialize HX711 task to read load cell values
    xTaskCreate(&hx711_task, "hx711_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

}
