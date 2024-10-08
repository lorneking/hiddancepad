#include <stdlib.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "gpio_control.h"
#include "hx711.h"
#include "wifi.h"
#include "web_server.h"
#include "esp_event.h"
#include "esp_task_wdt.h"
#include "serial_prompt.h"
#include "FreeRTOSConfig.h"
#include "hid_control.h"
#include "touchpad.h"

// #define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *TAG = "MAIN";

// Define GPIO Pins
#define HX711_SCK       GPIO_NUM_8 // Common clock pin for all HX711s
#define HX711_1_DT      GPIO_NUM_4 // Data pin for HX711 1
#define HX711_2_DT      GPIO_NUM_5 // Data pin for HX711 2
#define HX711_3_DT      GPIO_NUM_6 // Data pin for HX711 3
#define HX711_4_DT      GPIO_NUM_7 // Data pin for HX711 4
#define LED_1_GATE      GPIO_NUM_35 // Gate for LED 1
#define LED_2_GATE      GPIO_NUM_36 // Gate for LED 2
#define LED_3_GATE      GPIO_NUM_37 // Gate for LED 3
#define LED_4_GATE      GPIO_NUM_38 // Gate for LED 4

#define DOWN_ARROW_LED  LED_1_GATE // DOWN ARROW is LED/HX711 1
#define RIGHT_ARROW     LED_2_GATE // RIGHT ARROW is LED/HX711 2
#define UP_ARROW        LED_3_GATE // UP ARROW is LED/HX711 3
#define LEFT_ARROW      LED_4_GATE // LEFT ARROW is LED/HX711 4

#define DOWN_ARROW_HX711  HX711_1_DT // DOWN ARROW is LED/HX711 1
#define RIGHT_ARROW_HX711 HX711_2_DT // RIGHT ARROW is LED/HX711 2
#define UP_ARROW_HX711    HX711_3_DT // UP ARROW is LED/HX711 3
#define LEFT_ARROW_HX711  HX711_4_DT // LEFT ARROW is LED/HX711 4

// Define threshold values
long threshold = 14000; // Delta threshold for pad step detection
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
//    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
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

// typedef enum {
//     MOUSE_DIR_RIGHT,
//     MOUSE_DIR_DOWN,
//     MOUSE_DIR_DOWN,
//     MOUSE_DIR_LEFT,
//     MOUSE_DIR_UP,
//     MOUSE_DIR_MAX,
// } mouse_dir_t;

// #define DISTANCE_MAX        125
// #define DELTA_SCALAR        5

// static void mouse_draw_square_next_delta(int8_t *delta_x_ret, int8_t *delta_y_ret)
// {
//     static mouse_dir_t cur_dir = MOUSE_DIR_RIGHT;
//     static uint32_t distance = 0;

//     // Calculate next delta
//     if (cur_dir == MOUSE_DIR_RIGHT) {
//         *delta_x_ret = DELTA_SCALAR;
//         *delta_y_ret = 0;
//     } else if (cur_dir == MOUSE_DIR_DOWN) {
//         *delta_x_ret = 0;
//         *delta_y_ret = DELTA_SCALAR;
//     } else if (cur_dir == MOUSE_DIR_LEFT) {
//         *delta_x_ret = -DELTA_SCALAR;
//         *delta_y_ret = 0;
//     } else if (cur_dir == MOUSE_DIR_UP) {
//         *delta_x_ret = 0;
//         *delta_y_ret = -DELTA_SCALAR;
//     }

//     // Update cumulative distance for current direction
//     distance += DELTA_SCALAR;
//     // Check if we need to change direction
//     if (distance >= DISTANCE_MAX) {
//         distance = 0;
//         cur_dir++;
//         if (cur_dir == MOUSE_DIR_MAX) {
//             cur_dir = 0;
//         }
//     }
// }

// static void app_send_hid_keypress(uint8_t keycode)
// {
//     ESP_LOGI(TAG, "Sending Keyboard report");
//     uint8_t keycode_buf[6] = {keycode};
//     tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode_buf);
//     vTaskDelay(pdMS_TO_TICKS(13));
//     tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
// }

//static void app_send_mouse_move(int8_t delta_x, int8_t delta_y)
//{
//     ESP_LOGI(TAG, "Sending Mouse report");
//     tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, delta_x, delta_y, 0, 0);
//     vTaskDelay(pdMS_TO_TICKS(20));
// }

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
    hx711_init(&scale1, DOWN_ARROW_HX711, HX711_SCK, 64);
    hx711_init(&scale2, RIGHT_ARROW_HX711, HX711_SCK, 64);
    hx711_init(&scale3, UP_ARROW_HX711, HX711_SCK, 64);
    hx711_init(&scale4, LEFT_ARROW_HX711, HX711_SCK, 64);
    /* End 64x amplification */

    /* Begin 32x amplification */
    // hx711_init(&scale1, HX711_1_DT, HX711_SCK, 32);
    // hx711_init(&scale2, HX711_2_DT, HX711_SCK, 32);
    // hx711_init(&scale3, HX711_3_DT, HX711_SCK, 32);
    // hx711_init(&scale4, HX711_4_DT, HX711_SCK, 32);
    /* End 32x amplification */

    // Tare the scales after initialization
    hx711_tare(&scale1, 10);
    hx711_tare(&scale2, 10);
    hx711_tare(&scale3, 10);
    hx711_tare(&scale4, 10);

    // static bool send_hid_data = false;
    static bool send_hid_data = true;

    while (1) {      

        long weight1 = hx711_read(&scale1);
        long weight2 = hx711_read(&scale2);
        long weight3 = hx711_read(&scale3);
        long weight4 = hx711_read(&scale4);

        // ESP_LOGI(TAG, "Weight 1: %ld", weight1);
        // ESP_LOGI(TAG, "Weight 2: %ld", weight2);
        // ESP_LOGI(TAG, "Weight 3: %ld", weight3);
        // ESP_LOGI(TAG, "Weight 4: %ld", weight4);

        // if (tud_mounted()) {
        //     send_hid_data = true;
        // }

        if (weight1 != -1 && (weight1 - threshold) > prevWeight1) {
            gpio_set_level(LED_1_GATE, 1);
            ESP_LOGI(TAG, "Pad 1 step detected");
            if (send_hid_data) {
                send_hid_keypress(HID_KEY_ARROW_UP);
            }
        } else {
            gpio_set_level(LED_1_GATE, 0);
        }
        prevWeight1 = weight1;

        if (weight2 != -1 && (weight2 - threshold) > prevWeight2) {
            gpio_set_level(LED_2_GATE, 1);
            ESP_LOGI(TAG, "Pad 2 step detected");
            if (send_hid_data) {
                send_hid_keypress(HID_KEY_ARROW_DOWN);
            }
        } else {
            gpio_set_level(LED_2_GATE, 0);
        }
        prevWeight2 = weight2;

        if (weight3 != -1 && (weight3 - threshold) > prevWeight3) {
            gpio_set_level(LED_3_GATE, 1);
            ESP_LOGI(TAG, "Pad 3 step detected");
            if (send_hid_data) {
                send_hid_keypress(HID_KEY_ARROW_LEFT);
            }
        } else {
            gpio_set_level(LED_3_GATE, 0);
        }
        prevWeight3 = weight3;

        if (weight4 != -1 && (weight4 - threshold) > prevWeight4) {
            gpio_set_level(LED_4_GATE, 1);
            ESP_LOGI(TAG, "Pad 4 step detected");
            if (send_hid_data) {
                send_hid_keypress(HID_KEY_ARROW_RIGHT);
            }
        } else {
            gpio_set_level(LED_4_GATE, 0);
        }
        prevWeight4 = weight4;

        // Delay to match the HX711 sample rate (80Hz = 12.5ms per sample)
        vTaskDelay(pdMS_TO_TICKS(13));

        // send_hid_data = ((weight1 - threshold) > prevWeight1) || ((weight2 - threshold) > prevWeight2) || ((weight3 - threshold) > prevWeight3) || ((weight4 - threshold) > prevWeight4);
        // send_hid_data = tud_mounted();
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

    // Initialize UART for serial prompt
    uart_init();

    // Initialize LED driver GPIOs
    init_gpio(LED_1_GATE, GPIO_MODE_OUTPUT, GPIO_PULLUP_ONLY);
    init_gpio(LED_2_GATE, GPIO_MODE_OUTPUT, GPIO_PULLUP_ONLY);
    init_gpio(LED_3_GATE, GPIO_MODE_OUTPUT, GPIO_PULLUP_ONLY);
    init_gpio(LED_4_GATE, GPIO_MODE_OUTPUT, GPIO_PULLUP_ONLY);

    // Initialize HX711 task to read load cell values
    // xTaskCreate(&hx711_task, "hx711_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

    // xTaskCreate(&serial_prompt_task, "serial_prompt_task", 2048, NULL, 10, NULL);
    // xTaskCreate(&hx711_task, "hx711_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
 if (que_touch == NULL) {
        que_touch = xQueueCreate(TOUCH_BUTTON_NUM, sizeof(touch_event_t));
    }
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing touch pad");
    /* Initialize touch pad peripheral. */
    touch_pad_init();
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        touch_pad_config(button[i]);
    }

#if TOUCH_CHANGE_CONFIG
    /* If you want to change the touch sensor default setting, please write here (after initialization). */
    touch_pad_set_measurement_interval(TOUCH_PAD_SLEEP_CYCLE_DEFAULT);
    touch_pad_set_charge_discharge_times(TOUCH_PAD_MEASURE_CYCLE_DEFAULT);
    touch_pad_set_voltage(TOUCH_PAD_HIGH_VOLTAGE_THRESHOLD, TOUCH_PAD_LOW_VOLTAGE_THRESHOLD, TOUCH_PAD_ATTEN_VOLTAGE_THRESHOLD);
    touch_pad_set_idle_channel_connect(TOUCH_PAD_IDLE_CH_CONNECT_DEFAULT);
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        touch_pad_set_cnt_mode(button[i], TOUCH_PAD_SLOPE_DEFAULT, TOUCH_PAD_TIE_OPT_DEFAULT);
    }
#endif

#if TOUCH_BUTTON_DENOISE_ENABLE
    /* Denoise setting at TouchSensor 0. */
    touch_pad_denoise_t denoise = {
        /* The bits to be canceled are determined according to the noise level. */
        .grade = TOUCH_PAD_DENOISE_BIT4,
        /* By adjusting the parameters, the reading of T0 should be approximated to the reading of the measured channel. */
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
    };
    touch_pad_denoise_set_config(&denoise);
    touch_pad_denoise_enable();
    ESP_LOGI(TAG, "Denoise function init");
#endif

#if TOUCH_BUTTON_WATERPROOF_ENABLE
    /* Waterproof function */
    touch_pad_waterproof_t waterproof = {
        .guard_ring_pad = button[3],   // If no ring pad, set 0;
        /* Estimate the size of the parasitic capacitance on T14 and set appropriate hardware parameters. */
        .shield_driver = TOUCH_PAD_SHIELD_DRV_L2,
    };
    touch_pad_waterproof_set_config(&waterproof);
    touch_pad_waterproof_enable();
    ESP_LOGI(TAG, "touch pad waterproof init");
#endif

    /* Filter setting */
    touchsensor_filter_set(TOUCH_PAD_FILTER_IIR_16);
    touch_pad_timeout_set(true, TOUCH_PAD_THRESHOLD_MAX);
    /* Register touch interrupt ISR, enable intr type. */
    touch_pad_isr_register(touchsensor_interrupt_cb, NULL, TOUCH_PAD_INTR_MASK_ALL);
    /* If you have other touch algorithms, get the measured value after the `TOUCH_PAD_INTR_MASK_SCAN_DONE` interrupt is generated. */
    touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE | TOUCH_PAD_INTR_MASK_TIMEOUT);

    /* Enable touch sensor clock. Work mode is "timer trigger". */
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();

    // Start a task to show what pads have been touched
    xTaskCreate(&tp_read_task, "touch_pad_read_task", 4096, NULL, 5, NULL);
    // xTaskCreate(&tp_print_task, "touch_pad_print_task", 4096, NULL, 5, NULL);


}
