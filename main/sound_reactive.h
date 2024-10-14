#ifndef SOUND_REACTIVE_H
#define SOUND_REACTIVE_H

#include <stdint.h>
#include <stddef.h>
#include "driver/i2s_std.h"

// I2S Configuration Macros
#define I2S_PORT                I2S_NUM_0
#define I2S_SAMPLE_RATE         16000      // Sample rate in Hz
#define I2S_SAMPLE_BITS         16         // Bits per sample
#define I2S_DMA_BUF_LEN         1024       // DMA buffer length
#define I2S_DMA_BUF_COUNT       4          // Number of DMA buffers

// GPIO Pins
#define I2S_SCK_PIN             41         // GPIO41
#define I2S_WS_PIN              42         // GPIO42
#define I2S_SD_PIN              40         // GPIO40

// Sound Level Threshold (Adjustable)
#define SOUND_LEVEL_THRESHOLD   1000       // Adjust this value as needed

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the I2S driver for the microphone.
 *
 * This function sets up the I2S peripheral with the specified configurations
 * to interface with the I2S microphone.
 */
void i2s_init(void);

/**
 * @brief Task function for sound detection and LED control.
 *
 * This FreeRTOS task reads audio data from the I2S microphone, calculates
 * the sound level, and controls the LEDs based on a configurable threshold.
 *
 * @param arg Pointer to task parameters (unused).
 */
void sound_detection_task(void *arg);

/**
 * @brief Turn on the pad LEDs.
 *
 * Implement this function to control the LEDs on your pad to turn them on.
 */
void i2s_leds_on(void);

/**
 * @brief Turn off the pad LEDs.
 *
 * Implement this function to control the LEDs on your pad to turn them off.
 */
void i2s_leds_off(void);
float hanning_window(int n, int N);


#ifdef __cplusplus
}
#endif

#endif // SOUND_REACTIVE_LEDS_H
