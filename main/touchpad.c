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

static const char *TAG = "Touch pad";

QueueHandle_t que_touch = NULL;  // Declare the global variable for the touch event queue

// // Define touch pad used and list corresponding buttons
// const touch_pad_t button[TOUCH_BUTTON_NUM] = {
//     TOUCH_PAD_NUM4,     // 'UP' arrow.
//     TOUCH_PAD_NUM5,     // 'RIGHT' arrow.
//     TOUCH_PAD_NUM6,     // 'LEFT' arrow.
//     TOUCH_PAD_NUM7,     // 'DOWN' arrow.
// };

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

const float button_threshold[TOUCH_BUTTON_NUM] = {
    0.2, // 20%.
    0.2, // 20%.
    0.2, // 20%.
    0.1, // 20%.
    0.1, // 20%.
    0.1, // 20%.
    0.1, // 20%.
    0.2, // 20%.
    0.2, // 20%.
    0.2, // 20%.
    0.2, // 20%.
    0.2, // 20%.
    0.2, // 20%.
    0.2, // 20%
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

// void touchpad_query(void)
// {
//     if (que_touch == NULL) {
//         que_touch = xQueueCreate(TOUCH_BUTTON_NUM, sizeof(touch_event_t));
//     }
//     // Initialize touch pad peripheral, it will start a timer to run a filter
//     ESP_LOGI(TAG, "Initializing touch pad");
//     /* Initialize touch pad peripheral. */
//     touch_pad_init();
//     for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
//         touch_pad_config(button[i]);
//     }

// #if TOUCH_CHANGE_CONFIG
//     /* If you want to change the touch sensor default setting, please write here (after initialization). */
//     touch_pad_set_measurement_interval(TOUCH_PAD_SLEEP_CYCLE_DEFAULT);
//     touch_pad_set_charge_discharge_times(TOUCH_PAD_MEASURE_CYCLE_DEFAULT);
//     touch_pad_set_voltage(TOUCH_PAD_HIGH_VOLTAGE_THRESHOLD, TOUCH_PAD_LOW_VOLTAGE_THRESHOLD, TOUCH_PAD_ATTEN_VOLTAGE_THRESHOLD);
//     touch_pad_set_idle_channel_connect(TOUCH_PAD_IDLE_CH_CONNECT_DEFAULT);
//     for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
//         touch_pad_set_cnt_mode(button[i], TOUCH_PAD_SLOPE_DEFAULT, TOUCH_PAD_TIE_OPT_DEFAULT);
//     }
// #endif

// #if TOUCH_BUTTON_DENOISE_ENABLE
//     /* Denoise setting at TouchSensor 0. */
//     touch_pad_denoise_t denoise = {
//         /* The bits to be canceled are determined according to the noise level. */
//         .grade = TOUCH_PAD_DENOISE_BIT4,
//         /* By adjusting the parameters, the reading of T0 should be approximated to the reading of the measured channel. */
//         .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
//     };
//     touch_pad_denoise_set_config(&denoise);
//     touch_pad_denoise_enable();
//     ESP_LOGI(TAG, "Denoise function init");
// #endif

// #if TOUCH_BUTTON_WATERPROOF_ENABLE
//     /* Waterproof function */
//     touch_pad_waterproof_t waterproof = {
//         .guard_ring_pad = button[3],   // If no ring pad, set 0;
//         /* Estimate the size of the parasitic capacitance on T14 and set appropriate hardware parameters. */
//         .shield_driver = TOUCH_PAD_SHIELD_DRV_L2,
//     };
//     touch_pad_waterproof_set_config(&waterproof);
//     touch_pad_waterproof_enable();
//     ESP_LOGI(TAG, "touch pad waterproof init");
// #endif

//     /* Filter setting */
//     touchsensor_filter_set(TOUCH_PAD_FILTER_IIR_16);
//     touch_pad_timeout_set(true, TOUCH_PAD_THRESHOLD_MAX);
//     /* Register touch interrupt ISR, enable intr type. */
//     touch_pad_isr_register(touchsensor_interrupt_cb, NULL, TOUCH_PAD_INTR_MASK_ALL);
//     /* If you have other touch algorithms, get the measured value after the `TOUCH_PAD_INTR_MASK_SCAN_DONE` interrupt is generated. */
//     touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE | TOUCH_PAD_INTR_MASK_TIMEOUT);

//     /* Enable touch sensor clock. Work mode is "timer trigger". */
//     touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
//     touch_pad_fsm_start();

//     // Start a task to show what pads have been touched
//     xTaskCreate(&tp_example_read_task, "touch_pad_read_task", 4096, NULL, 5, NULL);
// }

#endif // PADS_USE_TOUCHPADS