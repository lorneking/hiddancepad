#include "config.h"

#if PADS_USE_LOAD_CELLS

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_defs.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hx711.h"

static const char *TAG = "HX711";

// Define threshold values
long threshold = 14000; // Delta threshold for pad step detection
long prevWeight1, prevWeight2, prevWeight3, prevWeight4;
HX711 scale1, scale2, scale3, scale4;

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

    }
}

#endif // PADS_USE_LOAD_CELLS
