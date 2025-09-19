#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "vl53l0x.h"
#include "flash.h"
#include "config.h"

static const char *TAG = "MAIN";

// Global Variables for calibration
volatile uint16_t lower_bound = 0;
volatile uint16_t upper_bound = 0;
volatile uint16_t true_upper_bound = 0;

// Convert to percentage
static uint16_t convert_to_percentage(uint16_t data) {
    if (data < lower_bound) {
        return 100;
    }
    if (data > upper_bound) {
        return 0;
    }
    
    uint16_t range = upper_bound - lower_bound;
    uint16_t position = upper_bound - data;
    uint16_t percentage = (position * 100) / range;
    
    return percentage;
}

// Calibration function
static void find_upper_and_lower(void) {
    ESP_LOGI(TAG, "Calibration Starting");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Please press pedal all the way down");
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Calculating...");
    lower_bound = read_range_continuous();
    ESP_LOGI(TAG, "Found Lower Bound: %d", lower_bound);

    ESP_LOGI(TAG, "Please ensure your foot is not on the pedal");
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Calculating...");
    upper_bound = read_range_continuous();
    ESP_LOGI(TAG, "Found Upper Bound: %d", upper_bound);
    
    true_upper_bound = upper_bound - lower_bound;
}

void app_main(void) {
    ESP_LOGI(TAG, "VL53L0X with Flash Storage");
    
    // Initialize flash
    esp_err_t ret = flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize flash!");
        return;
    }
    
    // Initialize I2C for VL53L0X
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    
    ESP_LOGI(TAG, "I2C initialized");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize VL53L0X sensor
    vl53l0x_init_simple();
    
    ESP_LOGI(TAG, "Starting continuous measurements...");
    start_continuous(100);  // 100ms period
    
    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for first measurement
    
    // Calibrate the sensor
    find_upper_and_lower();
    
    // Save calibration to flash
    uint8_t calib_data[4];
    calib_data[0] = lower_bound & 0xFF;
    calib_data[1] = (lower_bound >> 8) & 0xFF;
    calib_data[2] = upper_bound & 0xFF;
    calib_data[3] = (upper_bound >> 8) & 0xFF;
    
    ESP_LOGI(TAG, "Saving calibration to flash...");
    flash_write(0x000000, calib_data, 4);
    
    // Read back to verify
    uint8_t verify_data[4];
    flash_read(0x000000, verify_data, 4);
    uint16_t saved_lower = verify_data[0] | (verify_data[1] << 8);
    uint16_t saved_upper = verify_data[2] | (verify_data[3] << 8);
    ESP_LOGI(TAG, "Verified - Lower: %d, Upper: %d", saved_lower, saved_upper);
    
    // Main measurement loop
    while (1) {
        uint16_t range = read_range_continuous();
        uint8_t percentage = convert_to_percentage(range);
        ESP_LOGI(TAG, "Distance: %d mm, Percentage: %d%%", range, percentage);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}