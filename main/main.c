#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "vl53l0x.h"
#include "flash.h"
#include "config.h"

static const char *TAG = "MAIN";

// Debounce configuration
#define DEBOUNCE_TIME_MS 1000

// Global Variables for calibration
volatile uint16_t lower_bound = 0;
volatile uint16_t upper_bound = 0;
volatile uint16_t true_upper_bound = 0;

// Global flag for interrupt
volatile bool start_reading_data = false;
volatile bool calibration_flag = false;

// Timer handle for debouncing
static esp_timer_handle_t debounce_timer = NULL;

// Debounce timer callback - re-enables the interrupt
static void debounce_timer_callback(void* arg) {
    gpio_intr_enable(GPIO_NUM_1);
    gpio_intr_enable(GPIO_NUM_20);
}

// GPIO interrupt handler
static void IRAM_ATTR gpio_isr_handler_read_measurements(void* arg) {
    start_reading_data = !start_reading_data;
    gpio_intr_disable(GPIO_NUM_1);
    esp_timer_start_once(debounce_timer, DEBOUNCE_TIME_MS * 1000);
}

// Initialize debounce timer
static esp_err_t debounce_timer_init(void) {
    const esp_timer_create_args_t timer_args = {
        .callback = &debounce_timer_callback,
        .arg = NULL,
        .name = "debounce"
    };
    
    return esp_timer_create(&timer_args, &debounce_timer);
}
static void IRAM_ATTR gpio_isr_handler_start_calibration(void* arg) {
    calibration_flag = true;
    gpio_intr_disable(GPIO_NUM_20);
    esp_timer_start_once(debounce_timer, DEBOUNCE_TIME_MS * 1000);
}

// Init INTR
static esp_err_t interrupt_init(void) {
    // Configure GPIO1 for interrupt
    gpio_config_t io_conf_1 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t ret = gpio_config(&io_conf_1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO1");
        return ret;
    }
    
    ESP_LOGI(TAG, "GPIO1 interrupt configured");

    // Configure GPIO20 for interrupt
    gpio_config_t io_conf_20 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_20),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    ret = gpio_config(&io_conf_20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO20");
        return ret;
    }

    ESP_LOGI(TAG, "GPIO20 interrupt configured");

    // **INSTALL ISR SERVICE FIRST** - before adding any handlers
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return ret;
    }

    // Now add interrupt handlers
    ret = gpio_isr_handler_add(GPIO_NUM_1, gpio_isr_handler_read_measurements, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO1 ISR handler");
        return ret;
    }

    ret = gpio_isr_handler_add(GPIO_NUM_20, gpio_isr_handler_start_calibration, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO20 ISR handler");
        return ret;
    }

    return ESP_OK;
}

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

// Save calibration values to flash
static void save_calibration_to_flash(uint16_t lower, uint16_t upper) {
    uint8_t calib_data[4];
    
    // Pack the 16-bit values into bytes
    calib_data[0] = lower & 0xFF;           // Lower bound LSB
    calib_data[1] = (lower >> 8) & 0xFF;    // Lower bound MSB
    calib_data[2] = upper & 0xFF;           // Upper bound LSB
    calib_data[3] = (upper >> 8) & 0xFF;    // Upper bound MSB
    
    ESP_LOGI(TAG, "Saving calibration to flash - Lower: %d, Upper: %d", lower, upper);
    
    // Must erase sector before writing
    esp_err_t ret = flash_erase_sector(0x2000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase sector for calibration");
        return;
    }
    
    // Write to flash (address 0x2000)
    ret = flash_write(0x2000, calib_data, 4);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved successfully");
        
        // Read back to verify
        uint8_t verify_data[4];
        if (flash_read(0x2000, verify_data, 4) == ESP_OK) {
            uint16_t saved_lower = verify_data[0] | (verify_data[1] << 8);
            uint16_t saved_upper = verify_data[2] | (verify_data[3] << 8);
            ESP_LOGI(TAG, "Verification - Lower: %d, Upper: %d", saved_lower, saved_upper);
            
            if (saved_lower != lower || saved_upper != upper) {
                ESP_LOGE(TAG, "Verification failed! Data mismatch");
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to save calibration to flash");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "VL53L0X with Flash Storage");
    
    // Initialize flash
    esp_err_t ret = flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize flash!");
        return;
    }
    
    // Initialize debounce timer
    ret = debounce_timer_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize debounce timer!");
        return;
    }
    
    // Initialize interrupts
    ret = interrupt_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize interrupts!");
        return;
    }
    
    test_read_write();
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
    

    
    // Main measurement loop
    while (1) {
        if(calibration_flag){
                // Calibrate the sensor
                find_upper_and_lower();
                
                // Save calibration to flash using the new function
                save_calibration_to_flash(lower_bound, upper_bound);
                
                uint8_t calib_data[4];  // Array to hold 4 bytes
                flash_read(0x2000, calib_data, 4);

                // Reconstruct the 16-bit values
                uint16_t saved_lower = calib_data[0] | (calib_data[1] << 8);
                uint16_t saved_upper = calib_data[2] | (calib_data[3] << 8);

                ESP_LOGI(TAG, "Calibration from flash - Lower: %d, Upper: %d", saved_lower, saved_upper);
                calibration_flag = false;
                start_reading_data = false;
            }
        if(start_reading_data){
            uint16_t range = read_range_continuous();
            uint8_t percentage = convert_to_percentage(range);
            ESP_LOGI(TAG, "Distance: %d mm, Percentage: %d%%", range, percentage);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}