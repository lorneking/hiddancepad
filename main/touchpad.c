#include "config.h"

#if PADS_USE_TOUCHPADS

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/touch_pad.h"
#include "touchpad.h"
#include "gpio_control.h"
#include "config.h"
#include "pin_defs.h"
#include "hid_control.h"

static const char *TAG = "Touch pad";

QueueHandle_t que_touch = NULL;  // Declare the global variable for the touch event queue

const touch_pad_t button[TOUCH_BUTTON_NUM] = {
    TOUCH_PAD_NUM1,
    TOUCH_PAD_NUM2,
    TOUCH_PAD_NUM3,
    TOUCH_PAD_NUM4, // 'UP' arrow
    TOUCH_PAD_NUM5, // 'LEFT' arrow
    TOUCH_PAD_NUM6, // 'RIGHT' arrow
    TOUCH_PAD_NUM7, // 'DOWN' arrow
    TOUCH_PAD_NUM8,
    TOUCH_PAD_NUM9,
    TOUCH_PAD_NUM10,
    TOUCH_PAD_NUM11,
    TOUCH_PAD_NUM12,
    TOUCH_PAD_NUM13,
    TOUCH_PAD_NUM14
};

// Define button thresholds. A percentage raise of the benchmark value will trigger a corresponding button press.
// TODO: Allow these values to be calibrated and/or manually set through web interface.

const float button_threshold[TOUCH_BUTTON_NUM] = {
    0.2, // 20%. PAD 1
    0.2, // 20%. PAD 2
    0.2, // 20%. PAD 3
    0.07, // 7%. PAD 4 - UP arrow
    0.07, // 7%. PAD 5 - LEFT arrow
    0.07, // 7%. PAD 6 - RIGHT arrow
    0.07, // 7%. PAD 7 - DOWN arrow
    0.2, // 20%. PAD 8
    0.2, // 20%. PAD 9
    0.2, // 20%. PAD 10
    0.2, // 20%. PAD 11
    0.2, // 20%. PAD 12
    0.2, // 20%. PAD 13
    0.2, // 20%. PAD 14
};

void touchsensor_interrupt_cb(void *arg)
{
    int task_awoken = pdFALSE;
    touch_event_t evt;

    evt.intr_mask = touch_pad_read_intr_status_mask();
    evt.pad_status = touch_pad_get_status();
    evt.pad_num = touch_pad_get_current_meas_channel();

    xQueueSendFromISR(que_touch, &evt, &task_awoken);
    if (task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void tp_set_thresholds(void)
{
    uint32_t touch_value;
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        // Read benchmark value
        touch_pad_read_benchmark(button[i], &touch_value);
        // Set interrupt threshold
        touch_pad_set_thresh(button[i], touch_value * button_threshold[i]);
        ESP_LOGI(TAG, "touch pad [%d] base %"PRIu32", thresh %"PRIu32, \
                 button[i], touch_value, (uint32_t)(touch_value * button_threshold[i]));
    }
}

void touchsensor_filter_set(touch_filter_mode_t mode)
{
    /* Filter function */
    touch_filter_config_t filter_info = {
        .mode = mode,           // Test jitter and filter 1/4.
        .debounce_cnt = 1,      // 1 time count.
        .noise_thr = 0,         // 50%
        .jitter_step = 4,       // use for jitter mode.
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2,
    };
    touch_pad_filter_set_config(&filter_info);
    touch_pad_filter_enable();
    ESP_LOGI(TAG, "touch pad filter init");
}

void tp_read_task(void *pvParameter)
{
    touch_event_t evt = {0};
    static uint8_t guard_mode_flag = 0;
    /* Wait touch sensor init done */
    vTaskDelay(50 / portTICK_PERIOD_MS);
    tp_set_thresholds();

    while (1) {
        int ret = xQueueReceive(que_touch, &evt, (TickType_t)portMAX_DELAY);
        if (ret != pdTRUE) {
            continue;
        }

        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
            /* if guard pad be touched, other pads no response. */
            if (evt.pad_num == button[14]) {
                guard_mode_flag = 1;
                ESP_LOGW(TAG, "TouchSensor [%"PRIu32"] be activated, enter guard mode", evt.pad_num);
            } else {
                if (guard_mode_flag == 0) {
                    ESP_LOGI(TAG, "TouchSensor [%"PRIu32"] be activated, status mask 0x%"PRIu32"", evt.pad_num, evt.pad_status);
                    pad_led_on(evt.pad_num);
                    start_hid_keypress(pad_to_keycode(evt.pad_num));
                    ESP_LOGI(TAG, "LED ON: 0x%"PRIu32, evt.pad_num);
                } else {
                    ESP_LOGW(TAG, "In guard mode. No response");
                }
            }
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
            /* if guard pad be touched, other pads no response. */
            if (evt.pad_num == button[14]) {
                guard_mode_flag = 0;
                ESP_LOGW(TAG, "TouchSensor [%"PRIu32"] be inactivated, exit guard mode", evt.pad_num);
            } else {
                if (guard_mode_flag == 0) {
                    ESP_LOGI(TAG, "TouchSensor [%"PRIu32"] be inactivated, status mask 0x%"PRIu32, evt.pad_num, evt.pad_status);
                    pad_led_off(evt.pad_num);
                    end_hid_keypress(pad_to_keycode(evt.pad_num));
                    ESP_LOGI(TAG, "LED OFF: 0x%"PRIu32, evt.pad_num);
                }
            }
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_SCAN_DONE) {
            ESP_LOGI(TAG, "The touch sensor group measurement is done [%"PRIu32"].", evt.pad_num);
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_TIMEOUT) {
            /* Add your exception handling in here. */
            ESP_LOGI(TAG, "Touch sensor channel %"PRIu32" measure timeout. Skip this exception channel!!", evt.pad_num);
            touch_pad_timeout_resume(); // Point on the next channel to measure.
        }
    }
}

void tp_print_task(void *pvParameter)
{
    uint32_t touch_value;

    /* Wait touch sensor init done */
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("Touch Sensor read, the output format is: \nTouchpad num:[raw data]\n\n");

    while (1) {
        for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
            touch_pad_read_raw_data(button[i], &touch_value);    // read raw data.
            printf("T%d: [%4"PRIu32"] ", button[i], touch_value);
        }
        printf("\n");
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

uint32_t tp_read(touch_pad_t pad)
{
    uint32_t touch_value;
    touch_pad_read_raw_data(pad, &touch_value);
    return touch_value;
}

#endif // PADS_USE_TOUCHPADS