/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef TOUCHPAD_H
#define TOUCHPAD_H

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

#ifdef __cplusplus
extern "C" {
#endif

// Number of touch buttons
#define TOUCH_BUTTON_NUM 14

// Enable waterproof and denoise functionality
#define TOUCH_BUTTON_WATERPROOF_ENABLE 0
#define TOUCH_BUTTON_DENOISE_ENABLE 1

// Option to change touch sensor default settings
#define TOUCH_CHANGE_CONFIG 0

// Touch pad buttons used
extern const touch_pad_t button[TOUCH_BUTTON_NUM];

// Threshold values for each touch button
extern const float button_threshold[TOUCH_BUTTON_NUM];

// Queue handle for touch events
extern QueueHandle_t que_touch;

// Structure representing a touch event
typedef struct touch_msg {
    touch_pad_intr_mask_t intr_mask;
    uint32_t pad_num;
    uint32_t pad_status;
    uint32_t pad_val;
} touch_event_t;

// Function to set thresholds for touch pads
void tp_set_thresholds(void);

// Function to configure and enable touch pad filters
void touchsensor_filter_set(touch_filter_mode_t mode);

// Task function for reading touch pad events
void tp_read_task(void *pvParameter);

// Task function for printing touch pad values
void tp_print_task(void *pvParameter);

// Callback function for touch sensor interrupt handling
void touchsensor_interrupt_cb(void *arg);

// Function to initialize and start the touch pad query task
// void touchpad_query(void);

#ifdef __cplusplus
}
#endif

#endif // TOUCHPAD_H
