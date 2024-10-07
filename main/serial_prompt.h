#ifndef SERIAL_PROMPT_H
#define SERIAL_PROMPT_H

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "serial_prompt.h"
#include "esp_timer.h"
#include "FreeRTOSConfig.h"

// #define UART_NUM UART_NUM_1
#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)

void uart_init();
void simulate_keypress(char key);
void serial_prompt_task(void *arg);

#endif // SERIAL_PROMPT_H