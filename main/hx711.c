#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hx711.h"

#define TAG "HX711"

// Define macros for GPIO operations
#define digitalWrite(pin, level) gpio_set_level(pin, level)
#define digitalRead(pin) gpio_get_level(pin)
#define pinMode(pin, mode) gpio_set_direction(pin, mode)
#define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define delayMicroseconds(us) esp_rom_delay_us(us)  // Use ESP32 ROM delay function

// Initialize the HX711
void hx711_init(HX711 *hx711, gpio_num_t dout, gpio_num_t pd_sck, uint8_t gain) {
    hx711->PD_SCK = pd_sck;
    hx711->DOUT = dout;

    pinMode(hx711->PD_SCK, GPIO_MODE_OUTPUT);
    pinMode(hx711->DOUT, GPIO_MODE_INPUT);

    hx711_set_gain(hx711, gain);
}

// Check if HX711 is ready
bool hx711_is_ready(HX711 *hx711) {
    if (hx711 == NULL) {
        ESP_LOGE(TAG, "hx711 pointer is NULL");
        return false;
    }
    return digitalRead(hx711->DOUT) == 0;
}

// Set gain
void hx711_set_gain(HX711 *hx711, uint8_t gain) {
    switch (gain) {
        case 128:
            hx711->GAIN = GAIN_128;
            ESP_LOGI(TAG, "Gain set to 128x");
            break;
        case 64:
            hx711->GAIN = GAIN_64;
            ESP_LOGI(TAG, "Gain set to 64x");
            break;
        case 32:
            hx711->GAIN = GAIN_32;
            ESP_LOGI(TAG, "Gain set to 32x");
            break;
        default:
            ESP_LOGE(TAG, "Invalid gain value");
            break;
    }
}

// Read data from HX711
long hx711_read(HX711 *hx711) {
    if (hx711 == NULL) {
        ESP_LOGE(TAG, "hx711 pointer is NULL");
        return -1;
    }

    hx711_wait_ready(hx711, 0);

    uint32_t value = 0;
    uint8_t data[3] = {0};
    int32_t filler = 0x00;

    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);

    ESP_LOGD(TAG, "Reading data from HX711");

    // Read 24 bits of data from the HX711
    data[2] = hx711_shift_in_slow(hx711->DOUT, hx711->PD_SCK, MSBFIRST);
    data[1] = hx711_shift_in_slow(hx711->DOUT, hx711->PD_SCK, MSBFIRST);
    data[0] = hx711_shift_in_slow(hx711->DOUT, hx711->PD_SCK, MSBFIRST);

    // Debug print to verify data read
    ESP_LOGD(TAG, "Data read: %02x %02x %02x", data[2], data[1], data[0]);

    // Set gain for the next reading by clocking the device
    for (unsigned int i = 0; i < hx711->GAIN; i++) {
        digitalWrite(hx711->PD_SCK, 1);
        delayMicroseconds(1);
        digitalWrite(hx711->PD_SCK, 0);
        delayMicroseconds(1);
    }

    portEXIT_CRITICAL(&mux);

    // Perform sign extension for the 24-bit signed value
    if (data[2] & 0x80) {
        filler = 0xFF;  // Extend sign for negative values
    } else {
        filler = 0x00;
    }

    // Combine the 24-bit data into a signed 32-bit integer
    value = ((uint32_t)filler << 24
            | (uint32_t)data[2] << 16
            | (uint32_t)data[1] << 8
            | (uint32_t)data[0]);

    // Cast value to signed long
    // ESP_LOGI(TAG, "Raw value: %ld", (long)value);

    return (long)value;
}

// Wait for HX711 to be ready
void hx711_wait_ready(HX711 *hx711, unsigned long delay_ms) {
    while (!hx711_is_ready(hx711)) {
        // ESP_LOGW(TAG, "HX711 not ready, waiting...");
        delay(delay_ms);
    }
    // ESP_LOGI(TAG, "HX711 is ready");
}

// Retry waiting for HX711 to be ready
bool hx711_wait_ready_retry(HX711 *hx711, int retries, unsigned long delay_ms) {
    int count = 0;
    while (count < retries) {
        if (hx711_is_ready(hx711)) {
            return true;
        }
        delay(delay_ms);
        count++;
    }
    return false;
}

// Wait for HX711 to be ready with timeout
bool hx711_wait_ready_timeout(HX711 *hx711, unsigned long timeout, unsigned long delay_ms) {
    unsigned long start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout)) {
        if (hx711_is_ready(hx711)) {
            return true;
        }
        delay(delay_ms);
    }
    return false;
}

// Read average value
long hx711_read_average(HX711 *hx711, uint8_t times) {
    long sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += hx711_read(hx711);
        delay(0);
    }
    return sum / times;
}

// Get value
double hx711_get_value(HX711 *hx711, uint8_t times) {
    return hx711_read_average(hx711, times) - hx711->OFFSET;
}

// Get units
float hx711_get_units(HX711 *hx711, uint8_t times) {
    return hx711_get_value(hx711, times) / hx711->SCALE;
}

// Tare
void hx711_tare(HX711 *hx711, uint8_t times) {
    double sum = hx711_read_average(hx711, times);
    hx711_set_offset(hx711, sum);
}

// Set scale
void hx711_set_scale(HX711 *hx711, float scale) {
    hx711->SCALE = scale;
}

// Get scale
float hx711_get_scale(HX711 *hx711) {
    return hx711->SCALE;
}

// Set offset
void hx711_set_offset(HX711 *hx711, long offset) {
    hx711->OFFSET = offset;
}

// Get offset
long hx711_get_offset(HX711 *hx711) {
    return hx711->OFFSET;
}

// Power down
void hx711_power_down(HX711 *hx711) {
    digitalWrite(hx711->PD_SCK, 0);
    digitalWrite(hx711->PD_SCK, 1);
}

// Power up
void hx711_power_up(HX711 *hx711) {
    digitalWrite(hx711->PD_SCK, 0);
}

// Shift in data with speed support
uint8_t hx711_shift_in_slow(gpio_num_t dataPin, gpio_num_t clockPin, uint8_t bitOrder) {
    uint8_t value = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        digitalWrite(clockPin, 1);
        delayMicroseconds(1);
        if (bitOrder == LSBFIRST) {
            value |= digitalRead(dataPin) << i;
        } else {
            value |= digitalRead(dataPin) << (7 - i);
        }
        digitalWrite(clockPin, 0);
        delayMicroseconds(1);
    }
    return value;
}
