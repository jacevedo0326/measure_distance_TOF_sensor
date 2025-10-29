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
#include "bluetooth.h"

static const char *TAG = "MAIN";

// Debounce configuration
#define DEBOUNCE_TIME_MS 1000

// Flash storage configuration
#define CALIBRATION_FLASH_ADDR 0x2000
#define MEASUREMENTS_FLASH_ADDR 0x3000
#define FLASH_SIZE_BYTES (8 * 1024 * 1024)  // 64 Mbit = 8 MB
#define MAX_MEASUREMENTS ((FLASH_SIZE_BYTES - MEASUREMENTS_FLASH_ADDR) / sizeof(measurement_t))  // Use all available flash

// Global Variables for calibration
volatile uint16_t lower_bound = 0;
volatile uint16_t upper_bound = 0;
volatile uint16_t true_upper_bound = 0;

// Global flag for interrupt
volatile bool start_reading_data = false;
volatile bool calibration_flag = false;
volatile bool read_measurements_flag = false;
volatile bool erase_measurements_flag = false;
volatile bool download_data_flag = false;

// Measurement storage
typedef struct {
    uint16_t range;
    uint8_t percentage;
    uint8_t padding;  // For alignment
} measurement_t;

static uint32_t measurement_count = 0;

// Timer handle for debouncing
static esp_timer_handle_t debounce_timer = NULL;

// Function to log pin assignments
static void log_pin_assignments(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "PIN ASSIGNMENT MAP");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "I2C Pins:");
    ESP_LOGI(TAG, "  GPIO8  - I2C SDA (VL53L0X Data)");
    ESP_LOGI(TAG, "  GPIO9  - I2C SCL (VL53L0X Clock)");
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "Control Buttons:");
    ESP_LOGI(TAG, "  GPIO1  - Start/Stop Measurements");
    ESP_LOGI(TAG, "  GPIO20 - Start Calibration");
    ESP_LOGI(TAG, "  GPIO21 - Read Flash Measurements");
    ESP_LOGI(TAG, "  GPIO5  - Erase Flash Measurements");
    ESP_LOGI(TAG, "  GPIO6  - Download CSV Data");
    ESP_LOGI(TAG, "========================================");
}

// Debounce timer callback - re-enables the interrupt
static void debounce_timer_callback(void* arg) {
    gpio_intr_enable(GPIO_NUM_1);
    gpio_intr_enable(GPIO_NUM_20);
    gpio_intr_enable(GPIO_NUM_21);
    gpio_intr_enable(GPIO_NUM_5);
    gpio_intr_enable(GPIO_NUM_6);
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

static void IRAM_ATTR gpio_isr_handler_read_flash_measurements(void* arg) {
    read_measurements_flag = true;
    gpio_intr_disable(GPIO_NUM_21);
    esp_timer_start_once(debounce_timer, DEBOUNCE_TIME_MS * 1000);
}

static void IRAM_ATTR gpio_isr_handler_erase_measurements(void* arg) {
    erase_measurements_flag = true;
    gpio_intr_disable(GPIO_NUM_5);
    esp_timer_start_once(debounce_timer, DEBOUNCE_TIME_MS * 1000);
}

static void IRAM_ATTR gpio_isr_handler_download_data(void* arg) {
    download_data_flag = true;
    gpio_intr_disable(GPIO_NUM_6);
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
        ESP_LOGE(TAG, "Failed to configure GPIO1 (Start/Stop Measurements)");
        return ret;
    }
    
    ESP_LOGI(TAG, "GPIO1 configured - Start/Stop Measurements button");

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
        ESP_LOGE(TAG, "Failed to configure GPIO20 (Calibration)");
        return ret;
    }

    ESP_LOGI(TAG, "GPIO20 configured - Calibration button");

    // Configure GPIO21 for interrupt
    gpio_config_t io_conf_21 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_21),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    ret = gpio_config(&io_conf_21);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO21 (Read Measurements)");
        return ret;
    }

    ESP_LOGI(TAG, "GPIO21 configured - Read Flash Measurements button");

    // Configure GPIO5 for interrupt
    gpio_config_t io_conf_5 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_5),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    ret = gpio_config(&io_conf_5);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO5 (Erase Measurements)");
        return ret;
    }

    ESP_LOGI(TAG, "GPIO5 configured - Erase Flash Measurements button");

    // Configure GPIO6 for interrupt
    gpio_config_t io_conf_6 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_6),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    ret = gpio_config(&io_conf_6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO6 (Download CSV)");
        return ret;
    }

    ESP_LOGI(TAG, "GPIO6 configured - Download CSV Data button");

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

    ret = gpio_isr_handler_add(GPIO_NUM_21, gpio_isr_handler_read_flash_measurements, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO21 ISR handler");
        return ret;
    }

    ret = gpio_isr_handler_add(GPIO_NUM_5, gpio_isr_handler_erase_measurements, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO5 ISR handler");
        return ret;
    }

    ret = gpio_isr_handler_add(GPIO_NUM_6, gpio_isr_handler_download_data, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO6 ISR handler");
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
    esp_err_t ret = flash_erase_sector(CALIBRATION_FLASH_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase sector for calibration");
        return;
    }

    // Write to flash
    ret = flash_write(CALIBRATION_FLASH_ADDR, calib_data, 4);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved successfully");

        // Read back to verify
        uint8_t verify_data[4];
        if (flash_read(CALIBRATION_FLASH_ADDR, verify_data, 4) == ESP_OK) {
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

// Save a single measurement to flash
static void save_measurement(uint16_t range, uint8_t percentage) {
    if (measurement_count >= MAX_MEASUREMENTS) {
        ESP_LOGW(TAG, "Measurement buffer full, cannot save more");
        return;
    }

    measurement_t meas = {
        .range = range,
        .percentage = percentage,
        .padding = 0
    };

    uint32_t addr = MEASUREMENTS_FLASH_ADDR + (measurement_count * sizeof(measurement_t));

    // Erase sector if this is the first measurement in a new sector
    if (measurement_count % (FLASH_SECTOR_SIZE / sizeof(measurement_t)) == 0) {
        uint32_t sector_addr = MEASUREMENTS_FLASH_ADDR + (measurement_count * sizeof(measurement_t));
        flash_erase_sector(sector_addr);
    }

    esp_err_t ret = flash_write(addr, (uint8_t*)&meas, sizeof(measurement_t));
    if (ret == ESP_OK) {
        measurement_count++;
    } else {
        ESP_LOGE(TAG, "Failed to save measurement #%lu", measurement_count);
    }
}

// Read all measurements from flash
static void read_measurements_from_flash(void) {
    ESP_LOGI(TAG, "Reading %lu measurements from flash...", measurement_count);

    if (measurement_count == 0) {
        ESP_LOGI(TAG, "No measurements stored");
        return;
    }

    for (uint32_t i = 0; i < measurement_count; i++) {
        measurement_t meas;
        uint32_t addr = MEASUREMENTS_FLASH_ADDR + (i * sizeof(measurement_t));

        esp_err_t ret = flash_read(addr, (uint8_t*)&meas, sizeof(measurement_t));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Measurement #%lu: Range=%d mm, Percentage=%d%%", i + 1, meas.range, meas.percentage);
        } else {
            ESP_LOGE(TAG, "Failed to read measurement #%lu", i + 1);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to avoid overwhelming the output
    }

    ESP_LOGI(TAG, "Finished reading measurements");
}

// Erase all measurements from flash (keeps calibration data)
static void erase_measurements_from_flash(void) {
    ESP_LOGI(TAG, "Erasing all measurements from flash...");

    // Calculate how many sectors we need to erase
    uint32_t total_bytes = MAX_MEASUREMENTS * sizeof(measurement_t);
    uint32_t sectors_to_erase = (total_bytes + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;

    for (uint32_t i = 0; i < sectors_to_erase; i++) {
        uint32_t sector_addr = MEASUREMENTS_FLASH_ADDR + (i * FLASH_SECTOR_SIZE);
        esp_err_t ret = flash_erase_sector(sector_addr);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Erased sector at 0x%04lX", sector_addr);
        } else {
            ESP_LOGE(TAG, "Failed to erase sector at 0x%04lX", sector_addr);
        }
    }

    measurement_count = 0;
    ESP_LOGI(TAG, "All measurements erased. Measurement count reset to 0");
}

// Load calibration data from flash on startup
static void load_calibration_from_flash(void) {
    uint8_t calib_data[4];
    esp_err_t ret = flash_read(CALIBRATION_FLASH_ADDR, calib_data, 4);

    if (ret == ESP_OK) {
        uint16_t saved_lower = calib_data[0] | (calib_data[1] << 8);
        uint16_t saved_upper = calib_data[2] | (calib_data[3] << 8);

        // Check if flash contains valid data (not blank/erased)
        if (saved_lower != 0xFFFF && saved_upper != 0xFFFF && saved_lower < saved_upper) {
            lower_bound = saved_lower;
            upper_bound = saved_upper;
            true_upper_bound = upper_bound - lower_bound;
            ESP_LOGI(TAG, "Loaded calibration from flash - Lower: %d, Upper: %d", lower_bound, upper_bound);
        } else {
            ESP_LOGW(TAG, "No valid calibration found in flash. Please calibrate.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read calibration from flash");
    }
}

// Count existing measurements in flash on startup
static void count_measurements_in_flash(void) {
    measurement_count = 0;

    // Scan through flash to find how many valid measurements exist
    for (uint32_t i = 0; i < MAX_MEASUREMENTS; i++) {
        measurement_t meas;
        uint32_t addr = MEASUREMENTS_FLASH_ADDR + (i * sizeof(measurement_t));

        esp_err_t ret = flash_read(addr, (uint8_t*)&meas, sizeof(measurement_t));
        if (ret == ESP_OK) {
            // Check if this is a valid measurement (not blank flash)
            if (meas.range != 0xFFFF && meas.percentage != 0xFF) {
                measurement_count = i + 1;
            } else {
                // Found blank data, stop counting
                break;
            }
        } else {
            ESP_LOGE(TAG, "Error reading flash at measurement %lu", i);
            break;
        }
    }

    ESP_LOGI(TAG, "Found %lu existing measurements in flash", measurement_count);
}

// Download all measurements as CSV over serial
static void download_measurements_csv(void) {
    ESP_LOGI(TAG, "Starting CSV download of %lu measurements...", measurement_count);

    // Send CSV header with markers for parsing
    printf("\n===CSV_START===\n");
    printf("Index,Range_mm,Percentage\n");

    if (measurement_count == 0) {
        printf("===CSV_END===\n");
        ESP_LOGI(TAG, "No measurements to download");
        return;
    }

    // Send all measurements
    for (uint32_t i = 0; i < measurement_count; i++) {
        measurement_t meas;
        uint32_t addr = MEASUREMENTS_FLASH_ADDR + (i * sizeof(measurement_t));

        esp_err_t ret = flash_read(addr, (uint8_t*)&meas, sizeof(measurement_t));
        if (ret == ESP_OK) {
            printf("%lu,%u,%u\n", i + 1, meas.range, meas.percentage);
        } else {
            ESP_LOGE(TAG, "Failed to read measurement #%lu during download", i + 1);
        }
    }

    printf("===CSV_END===\n");
    ESP_LOGI(TAG, "CSV download complete. %lu measurements sent.", measurement_count);
}

void app_main(void) {
    ESP_LOGI(TAG, "VL53L0X with Flash Storage");
    
    // Log pin assignments at startup
    log_pin_assignments();
    
    // Initialize flash
    esp_err_t ret = flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize flash!");
        return;
    }

    // Load calibration and measurement count from flash
    load_calibration_from_flash();
    count_measurements_in_flash();

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

    // Initialize Bluetooth
    ret = bluetooth_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth!");
        return;
    }

    // test_read_write();  // Commented out - erases flash on startup
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
    
    ESP_LOGI(TAG, "I2C initialized on GPIO8 (SDA) and GPIO9 (SCL)");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize VL53L0X sensor
    vl53l0x_init_simple();
    
    ESP_LOGI(TAG, "Starting continuous measurements...");
    start_continuous(100);  // 100ms period
    
    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for first measurement
    // Start Bluetooth test
    ESP_LOGI(TAG, "Starting Bluetooth connection test...");
    test_bluetooth_connection();


    // Main measurement loop
    while (1) {
        if(calibration_flag){
                ESP_LOGI(TAG, "Calibration triggered via GPIO20");
                // Calibrate the sensor
                find_upper_and_lower();

                // Save calibration to flash using the new function
                save_calibration_to_flash(lower_bound, upper_bound);

                uint8_t calib_data[4];  // Array to hold 4 bytes
                flash_read(CALIBRATION_FLASH_ADDR, calib_data, 4);

                // Reconstruct the 16-bit values
                uint16_t saved_lower = calib_data[0] | (calib_data[1] << 8);
                uint16_t saved_upper = calib_data[2] | (calib_data[3] << 8);

                ESP_LOGI(TAG, "Calibration from flash - Lower: %d, Upper: %d", saved_lower, saved_upper);
                calibration_flag = false;
                start_reading_data = false;

                // Reset measurement count when recalibrating
                measurement_count = 0;
            }

        if(read_measurements_flag){
            ESP_LOGI(TAG, "Read measurements triggered via GPIO21");
            read_measurements_from_flash();
            read_measurements_flag = false;
        }

        if(erase_measurements_flag){
            ESP_LOGI(TAG, "Erase measurements triggered via GPIO5");
            erase_measurements_from_flash();
            erase_measurements_flag = false;
        }

        if(download_data_flag){
            ESP_LOGI(TAG, "CSV download triggered via GPIO6");
            download_measurements_csv();
            download_data_flag = false;
        }

        if(start_reading_data){
            uint16_t range = read_range_continuous();
            uint8_t percentage = convert_to_percentage(range);
            ESP_LOGI(TAG, "Distance: %d mm, Percentage: %d%%", range, percentage);

            // Save measurement to flash
            save_measurement(range, percentage);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}