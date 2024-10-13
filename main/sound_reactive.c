// sound_reactive.c

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dsp_platform.h"
#include "esp_dsp.h"

// Suppress deprecation warnings
#define I2S_SUPPRESS_DEPRECATION_WARN
#include "driver/i2s.h"

#include "esp_system.h"
#include "esp_log.h"

// Include your LED control functions
#include "gpio_control.h"

static const char *TAG = "SoundReactiveLEDs";

// I2S Configuration
#define I2S_PORT                I2S_NUM_0
#define I2S_SAMPLE_RATE         16000      // Sample rate in Hz
#define I2S_SAMPLE_BITS         32         // Bits per sample
#define I2S_DMA_BUF_LEN         1024       // DMA buffer length
#define I2S_DMA_BUF_COUNT       4          // Number of DMA buffers

// GPIO Pins (Verified with your connections)
#define I2S_SCK_PIN             41         // GPIO41 (BCK)
#define I2S_WS_PIN              42         // GPIO42 (WS)
#define I2S_SD_PIN              40         // GPIO40 (DATA IN)

// Cutoff Frequencies for Bass Detection
#define LOWER_CUTOFF_FREQUENCY   20   // Lower frequency in Hz
#define UPPER_CUTOFF_FREQUENCY   250  // Upper frequency in Hz

void i2s_init(void)
{
    // I2S configuration structure
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = I2S_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
    };

    // I2S pin configuration
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,      // Serial Clock (BCK)
        .ws_io_num = I2S_WS_PIN,        // Word Select (WS)
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN       // Serial Data (SD)
    };

    // Install and start I2S driver
    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pin_config));
    ESP_LOGI(TAG, "I2S driver installed");
}

float hanning_window(int n, int N)
{
    return 0.5f * (1.0f - cosf(2.0f * M_PI * n / (N - 1)));
}

void sound_detection_task(void *arg)
{
    size_t bytes_read;
    int N = I2S_DMA_BUF_LEN; // Number of samples

    int32_t *i2s_read_buff = (int32_t *)malloc(N * sizeof(int32_t));
    if (!i2s_read_buff) {
        ESP_LOGE(TAG, "Failed to allocate memory for I2S buffer");
        vTaskDelete(NULL);
    }

    // Buffer for FFT input (complex numbers)
    float *fft_input = (float *)malloc(sizeof(float) * 2 * N); // Real and Imaginary parts
    if (!fft_input) {
        ESP_LOGE(TAG, "Failed to allocate memory for FFT input buffer");
        free(i2s_read_buff);
        vTaskDelete(NULL);
    }

    while (1) {
        // Read data from I2S
        bytes_read = 0;
        esp_err_t result = i2s_read(I2S_PORT, (void *)i2s_read_buff, N * sizeof(int32_t), &bytes_read, portMAX_DELAY);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(result));
            continue;
        }

        int sample_count = bytes_read / sizeof(int32_t);
        if (sample_count < N) {
            ESP_LOGW(TAG, "Not enough samples read: %d", sample_count);
            continue;
        }

        // Print raw samples to verify microphone functionality
        // ESP_LOGI(TAG, "First 10 Raw Samples:");
        // for (int i = 0; i < 10 && i < sample_count; i++) {
        //     ESP_LOGI(TAG, "Sample [%d]: %ld", i, (long)i2s_read_buff[i]);
        // }

        // Convert int32_t samples to float and apply window function
        for (int i = 0; i < N; i++) {
            // Read the raw sample
            uint32_t sample = (uint32_t)i2s_read_buff[i];
            // Extract the 24-bit data (bits 31 to 8)
            int32_t raw_data = (int32_t)(sample >> 8);
            // Sign-extend the 24-bit data to 32 bits
            if (raw_data & 0x800000) { // Check if sign bit (bit 23) is set
                raw_data |= 0xFF000000; // Set upper bits to 1
            } else {
                raw_data &= 0x00FFFFFF; // Ensure upper bits are 0
            }
            // Convert to float and apply window function
            fft_input[i * 2] = (float)raw_data * hanning_window(i, N);
            fft_input[i * 2 + 1] = 0.0f; // Imaginary part set to zero
        }

        // Perform FFT
        dsps_fft2r_fc32(fft_input, N);
        dsps_bit_rev_fc32(fft_input, N);
        dsps_cplx2reC_fc32(fft_input, N);

        // Frequency resolution
        float freq_resolution = (float)I2S_SAMPLE_RATE / N;

        // Calculate index range for bass frequencies
        int i_lower = (int)((LOWER_CUTOFF_FREQUENCY / freq_resolution) + 0.5f);
        int i_upper = (int)((UPPER_CUTOFF_FREQUENCY / freq_resolution) + 0.5f);
        if (i_lower < 0) i_lower = 0;
        if (i_upper >= N / 2) i_upper = N / 2 - 1;

        // Initialize bass_sum and bass_count
        float bass_sum = 0.0f;
        int bass_count = 0;

        for (int i = i_lower; i <= i_upper; i++) {
            float real = fft_input[i * 2];
            float imag = fft_input[i * 2 + 1];
            float magnitude = sqrtf(real * real + imag * imag) / N; // Normalize
            bass_sum += magnitude;
            bass_count++;
        }

        // Calculate bass average
        float bass_average = 0.0f;
        if (bass_count > 0) {
            bass_average = bass_sum / bass_count;
        } else {
            ESP_LOGW(TAG, "bass_count is zero, cannot compute bass_average");
            bass_average = 0.0f;
        }

        // ESP_LOGI(TAG, "Bass Level: %.2f", bass_average);

        // Threshold for bass detection
        // float bass_threshold = 1000000.0f; // Adjust this value based on testing
        float bass_threshold = 700.0f;

        // Check if bass level exceeds the threshold
        if (bass_average > bass_threshold) {
            pad_led_on(0x4);   // Turn on LEDs
            pad_led_on(0x5);
            pad_led_on(0x6);
            pad_led_on(0x7);
        } else {
            pad_led_off(0x4);  // Turn off LEDs
            pad_led_off(0x5);
            pad_led_off(0x6);
            pad_led_off(0x7);
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Adjust delay as needed
    }

    free(i2s_read_buff);
    free(fft_input);
    vTaskDelete(NULL);
}


