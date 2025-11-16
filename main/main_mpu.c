#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mpu6500.h"
#include "flash.h"
#include "config.h"
#include "bluetooth.h"

static const char *TAG = "MAIN_MPU";

// Bluetooth command definitions
#define BLE_CMD_START_STOP      0x01  // Start/Stop measurements (GPIO1)
#define BLE_CMD_CALIBRATE       0x15  // Start calibration (21 in decimal - GPIO21)
#define BLE_CMD_READ_MEAS       0x03  // Read measurements (GPIO20)
#define BLE_CMD_ERASE_MEAS      0x04  // Erase measurements (GPIO5)
#define BLE_CMD_DOWNLOAD_CSV    0x05  // Download CSV data (GPIO6)

// Debounce configuration
#define DEBOUNCE_TIME_MS 1000

// Flash storage configuration
#define CALIBRATION_FLASH_ADDR 0x2000
#define MEASUREMENTS_FLASH_ADDR 0x3000
#define FLASH_SIZE_BYTES (8 * 1024 * 1024)  // 64 Mbit = 8 MB
#define MAX_MEASUREMENTS ((FLASH_SIZE_BYTES - MEASUREMENTS_FLASH_ADDR) / sizeof(measurement_t))

// Gyroscope configuration
#define GYRO_SAMPLE_PERIOD_MS 100  // 100ms = 10Hz sampling rate
#define GYRO_SAMPLE_PERIOD_S (GYRO_SAMPLE_PERIOD_MS / 1000.0f)

// Calibration configuration - gyro offset averaging
#define GYRO_CALIBRATION_SAMPLES 100
#define MIN_CALIBRATION_RANGE 5.0f  // Minimum acceptable range in degrees (reject if smaller)

// Global Variables for calibration
volatile float gyro_offset_x = 0.0f;
volatile float gyro_offset_y = 0.0f;
volatile float gyro_offset_z = 0.0f;
volatile float min_angle = 0.0f;       // Pedal fully released (rest position)
volatile float max_angle = 0.0f;       // Pedal fully pressed

// Current pedal state
volatile float current_angle = 0.0f;

// Global flags for interrupt
volatile bool start_reading_data = false;
volatile bool calibration_flag = false;
volatile bool read_measurements_flag = false;
volatile bool erase_measurements_flag = false;
volatile bool download_data_flag = false;

// Measurement storage
typedef struct {
    float angle;
    uint8_t percentage;
    uint8_t padding[3];  // For alignment to 8 bytes
} measurement_t;

static uint32_t measurement_count = 0;

// MPU6500 handle
static mpu6500_t mpu;

// Timer handle for debouncing
static esp_timer_handle_t debounce_timer = NULL;

// Function to log pin assignments
static void log_pin_assignments(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "PIN ASSIGNMENT MAP (GYROSCOPE MODE)");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "I2C Pins:");
    ESP_LOGI(TAG, "  GPIO8  - I2C SDA (MPU6500 Data)");
    ESP_LOGI(TAG, "  GPIO9  - I2C SCL (MPU6500 Clock)");
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "Control Buttons:");
    ESP_LOGI(TAG, "  GPIO1  - Start/Stop Measurements");
    ESP_LOGI(TAG, "  GPIO21 - Start Calibration");
    ESP_LOGI(TAG, "  GPIO20 - Read Flash Measurements");
    ESP_LOGI(TAG, "  GPIO5  - Erase Flash Measurements");
    ESP_LOGI(TAG, "  GPIO6  - Download CSV Data");
    ESP_LOGI(TAG, "========================================");
}

// Debounce timer callback - re-enables the interrupt
static void debounce_timer_callback(void* arg) {
    gpio_intr_enable(GPIO_NUM_1);
    gpio_intr_enable(GPIO_NUM_21);
    gpio_intr_enable(GPIO_NUM_20);
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
    gpio_intr_disable(GPIO_NUM_21);
    esp_timer_start_once(debounce_timer, DEBOUNCE_TIME_MS * 1000);
}

static void IRAM_ATTR gpio_isr_handler_read_flash_measurements(void* arg) {
    read_measurements_flag = true;
    gpio_intr_disable(GPIO_NUM_20);
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
    // Configure GPIO1 for interrupt (Start/Stop)
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

    // Configure GPIO21 for interrupt (Calibration)
    gpio_config_t io_conf_21 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_21),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    ret = gpio_config(&io_conf_21);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO21 (Calibration)");
        return ret;
    }

    ESP_LOGI(TAG, "GPIO21 configured - Calibration button");

    // Configure GPIO20 for interrupt (Read Measurements)
    gpio_config_t io_conf_20 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_20),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    ret = gpio_config(&io_conf_20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO20 (Read Measurements)");
        return ret;
    }

    ESP_LOGI(TAG, "GPIO20 configured - Read Flash Measurements button");

    // Configure GPIO5 for interrupt (Erase)
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

    // Configure GPIO6 for interrupt (Download CSV)
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

    ret = gpio_isr_handler_add(GPIO_NUM_21, gpio_isr_handler_start_calibration, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO21 ISR handler");
        return ret;
    }

    ret = gpio_isr_handler_add(GPIO_NUM_20, gpio_isr_handler_read_flash_measurements, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO20 ISR handler");
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

// Convert angle to percentage
static uint8_t convert_to_percentage(float angle) {
    // Handle both normal and reversed calibration
    bool reversed = (min_angle > max_angle);
    float actual_min = reversed ? max_angle : min_angle;
    float actual_max = reversed ? min_angle : max_angle;

    // Clamp to physical limits
    float clamped_angle = angle;
    if (clamped_angle < actual_min) clamped_angle = actual_min;
    if (clamped_angle > actual_max) clamped_angle = actual_max;

    // Calculate percentage
    float percentage;
    if (reversed) {
        // If reversed: min (released) is larger, max (pressed) is smaller
        percentage = ((min_angle - clamped_angle) / (min_angle - max_angle)) * 100.0f;
    } else {
        // Normal: as angle increases from min to max, percentage increases
        percentage = ((clamped_angle - min_angle) / (max_angle - min_angle)) * 100.0f;
    }

    // Clamp percentage to 0-100
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;

    return (uint8_t)percentage;
}

// Calibrate gyroscope - find zero offset and angle range
static void calibrate_gyroscope(void) {
    const char *msg;
    char buffer[128];

    msg = "Gyroscope Calibration Starting";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Step 1: Calibrate gyro offset (pedal must be stationary)
    msg = "Step 1: Calibrating gyro offset. Keep pedal STILL!";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Collect samples for offset calibration
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        mpu6500_scaled_data_t gyro;
        if (mpu6500_read_gyro(&mpu, &gyro) == ESP_OK) {
            sum_x += gyro.x;
            sum_y += gyro.y;
            sum_z += gyro.z;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    gyro_offset_x = sum_x / GYRO_CALIBRATION_SAMPLES;
    gyro_offset_y = sum_y / GYRO_CALIBRATION_SAMPLES;
    gyro_offset_z = sum_z / GYRO_CALIBRATION_SAMPLES;

    snprintf(buffer, sizeof(buffer), "Gyro offsets - X:%.2f Y:%.2f Z:%.2f deg/s",
             gyro_offset_x, gyro_offset_y, gyro_offset_z);
    ESP_LOGI(TAG, "%s", buffer);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)buffer, strlen(buffer));
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    // DIAGNOSTIC: Show all 3 axes to help user identify correct axis
    msg = "DIAGNOSTIC: Move pedal and watch which axis changes most";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }

    for (int i = 0; i < 50; i++) {  // 5 seconds of diagnostic data
        mpu6500_scaled_data_t gyro;
        if (mpu6500_read_gyro(&mpu, &gyro) == ESP_OK) {
            float corrected_x = gyro.x - gyro_offset_x;
            float corrected_y = gyro.y - gyro_offset_y;
            float corrected_z = gyro.z - gyro_offset_z;

            ESP_LOGI(TAG, "Gyro: X=%.2f  Y=%.2f  Z=%.2f deg/s",
                     corrected_x, corrected_y, corrected_z);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    msg = "Move pedal one more time and note the axis!";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    // Step 2: Calibrate min angle (pedal fully released)
    msg = "Step 2: Ensure pedal is FULLY RELEASED";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    msg = "Setting min angle (released position)...";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }

    // Reset current angle to 0 at rest position
    min_angle = 0.0f;
    current_angle = 0.0f;

    snprintf(buffer, sizeof(buffer), "Min Angle (Released): %.2f degrees", min_angle);
    ESP_LOGI(TAG, "%s", buffer);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)buffer, strlen(buffer));
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Step 3: Calibrate max angle (pedal fully pressed)
    msg = "Step 3: SLOWLY press pedal ALL THE WAY DOWN";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    msg = "Starting measurement in 3...";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    msg = "2...";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    msg = "1... NOW press SLOWLY!";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Integrate gyro for 10 seconds while user SLOWLY presses pedal
    // This gives much more time and shows real-time feedback
    float temp_angle = 0.0f;
    ESP_LOGI(TAG, "Press pedal SLOWLY over the next 10 seconds...");

    for (int i = 0; i < 100; i++) {  // 100 samples * 100ms = 10 seconds
        mpu6500_scaled_data_t gyro;
        if (mpu6500_read_gyro(&mpu, &gyro) == ESP_OK) {
            // Remove offset and integrate
            // CHANGED: Using X-axis instead of Y-axis based on diagnostic data
            float corrected_gyro_x = gyro.x - gyro_offset_x;
            temp_angle += corrected_gyro_x * GYRO_SAMPLE_PERIOD_S;

            // Show progress every second
            if (i % 10 == 0) {
                ESP_LOGI(TAG, "Current angle: %.2f deg, Gyro rate: %.2f deg/s",
                         temp_angle, corrected_gyro_x);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(GYRO_SAMPLE_PERIOD_MS));
    }

    max_angle = temp_angle;
    current_angle = max_angle;  // Set current position to max since pedal is pressed

    snprintf(buffer, sizeof(buffer), "Max Angle (Pressed): %.2f degrees", max_angle);
    ESP_LOGI(TAG, "%s", buffer);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)buffer, strlen(buffer));
    }

    // Validate calibration range
    float total_range = fabsf(max_angle - min_angle);
    if (total_range < MIN_CALIBRATION_RANGE) {
        snprintf(buffer, sizeof(buffer),
                 "WARNING: Range too small (%.2f deg)! Need at least %.1f deg. Recalibrate!",
                 total_range, MIN_CALIBRATION_RANGE);
        ESP_LOGW(TAG, "%s", buffer);
        if(bluetooth_is_connected()) {
            bluetooth_send_notification((const uint8_t *)buffer, strlen(buffer));
        }

        msg = "CALIBRATION FAILED - Range too small!";
        ESP_LOGE(TAG, "%s", msg);
        if(bluetooth_is_connected()) {
            bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
        }
        return;  // Don't save bad calibration
    }

    msg = "Calibration Complete!";
    ESP_LOGI(TAG, "%s", msg);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
    }

    snprintf(buffer, sizeof(buffer), "Range: %.2f to %.2f degrees (%.2f deg total)",
             min_angle, max_angle, total_range);
    ESP_LOGI(TAG, "%s", buffer);
    if(bluetooth_is_connected()) {
        bluetooth_send_notification((const uint8_t *)buffer, strlen(buffer));
    }
}

// Save calibration values to flash
static void save_calibration_to_flash(void) {
    uint8_t calib_data[20];  // 5 floats * 4 bytes

    // Pack float values as bytes
    memcpy(&calib_data[0], &gyro_offset_x, 4);
    memcpy(&calib_data[4], &gyro_offset_y, 4);
    memcpy(&calib_data[8], &gyro_offset_z, 4);
    memcpy(&calib_data[12], &min_angle, 4);
    memcpy(&calib_data[16], &max_angle, 4);

    ESP_LOGI(TAG, "Saving gyro calibration to flash");
    ESP_LOGI(TAG, "  Offsets: X=%.2f Y=%.2f Z=%.2f", gyro_offset_x, gyro_offset_y, gyro_offset_z);
    ESP_LOGI(TAG, "  Angles: Min=%.2f Max=%.2f", min_angle, max_angle);

    // Erase sector before writing
    esp_err_t ret = flash_erase_sector(CALIBRATION_FLASH_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase sector for calibration");
        return;
    }

    // Write to flash
    ret = flash_write(CALIBRATION_FLASH_ADDR, calib_data, 20);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved successfully");
    } else {
        ESP_LOGE(TAG, "Failed to save calibration to flash");
    }
}

// Load calibration data from flash on startup
static void load_calibration_from_flash(void) {
    uint8_t calib_data[20];
    esp_err_t ret = flash_read(CALIBRATION_FLASH_ADDR, calib_data, 20);

    if (ret == ESP_OK) {
        // Unpack float values
        float saved_offset_x, saved_offset_y, saved_offset_z;
        float saved_min_angle, saved_max_angle;

        memcpy(&saved_offset_x, &calib_data[0], 4);
        memcpy(&saved_offset_y, &calib_data[4], 4);
        memcpy(&saved_offset_z, &calib_data[8], 4);
        memcpy(&saved_min_angle, &calib_data[12], 4);
        memcpy(&saved_max_angle, &calib_data[16], 4);

        // Check if flash contains valid data (not blank/erased)
        // Check for NaN or unreasonable values
        if (!isnan(saved_offset_x) && !isnan(saved_offset_y) && !isnan(saved_offset_z) &&
            !isnan(saved_min_angle) && !isnan(saved_max_angle) &&
            fabsf(saved_offset_x) < 50.0f && fabsf(saved_offset_y) < 50.0f &&
            fabsf(saved_offset_z) < 50.0f &&
            fabsf(saved_min_angle - saved_max_angle) > 1.0f) {

            gyro_offset_x = saved_offset_x;
            gyro_offset_y = saved_offset_y;
            gyro_offset_z = saved_offset_z;
            min_angle = saved_min_angle;
            max_angle = saved_max_angle;
            current_angle = min_angle;  // Start at rest position

            ESP_LOGI(TAG, "Loaded gyro calibration from flash");
            ESP_LOGI(TAG, "  Offsets: X=%.2f Y=%.2f Z=%.2f", gyro_offset_x, gyro_offset_y, gyro_offset_z);
            ESP_LOGI(TAG, "  Angles: Min=%.2f Max=%.2f", min_angle, max_angle);
        } else {
            ESP_LOGW(TAG, "No valid gyro calibration found in flash. Please calibrate.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read calibration from flash");
    }
}

// Save a single measurement to flash
static void save_measurement(float angle, uint8_t percentage) {
    if (measurement_count >= MAX_MEASUREMENTS) {
        ESP_LOGW(TAG, "Measurement buffer full, cannot save more");
        return;
    }

    measurement_t meas = {
        .angle = angle,
        .percentage = percentage,
        .padding = {0, 0, 0}
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
            ESP_LOGI(TAG, "Measurement #%lu: Angle=%.2f deg, Percentage=%d%%",
                     i + 1, meas.angle, meas.percentage);
        } else {
            ESP_LOGE(TAG, "Failed to read measurement #%lu", i + 1);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
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
            if (!isnan(meas.angle) && meas.percentage != 0xFF) {
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
    printf("Index,Angle_deg,Percentage\n");

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
            printf("%lu,%.2f,%u\n", i + 1, meas.angle, meas.percentage);
        } else {
            ESP_LOGE(TAG, "Failed to read measurement #%lu during download", i + 1);
        }
    }

    printf("===CSV_END===\n");
    ESP_LOGI(TAG, "CSV download complete. %lu measurements sent.", measurement_count);
}

// Bluetooth data receive callback
static void bluetooth_data_received(const uint8_t *data, uint16_t len) {
    if (len > 0) {
        switch (data[0]) {
        case BLE_CMD_START_STOP:
            ESP_LOGI(TAG, "BLE Command: Start/Stop Measurements");
            start_reading_data = !start_reading_data;
            break;

        case BLE_CMD_CALIBRATE:
            ESP_LOGI(TAG, "BLE Command: Calibrate");
            calibration_flag = true;
            break;

        case BLE_CMD_READ_MEAS:
            ESP_LOGI(TAG, "BLE Command: Read Measurements");
            read_measurements_flag = true;
            break;

        case BLE_CMD_ERASE_MEAS:
            ESP_LOGI(TAG, "BLE Command: Erase Measurements");
            erase_measurements_flag = true;
            break;

        case BLE_CMD_DOWNLOAD_CSV:
            ESP_LOGI(TAG, "BLE Command: Download CSV");
            download_data_flag = true;
            break;

        default:
            ESP_LOGW(TAG, "Unknown BLE command: 0x%02X", data[0]);
            break;
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "MPU6500 Gyroscope Pedal Position Tracker");

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

    // Initialize Bluetooth with callback
    ret = bluetooth_init(bluetooth_data_received);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth!");
        return;
    }

    // Initialize I2C for MPU6500
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,  // 400kHz for MPU6500
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "I2C initialized on GPIO8 (SDA) and GPIO9 (SCL)");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize MPU6500 sensor
    ret = mpu6500_init(&mpu, I2C_NUM_0, MPU6500_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6500!");
        return;
    }

    // Set gyroscope to ±500°/s range (good for pedal motion)
    ret = mpu6500_set_gyro_fs(&mpu, MPU6500_GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope range!");
        return;
    }

    ESP_LOGI(TAG, "MPU6500 initialized and ready");
    ESP_LOGI(TAG, "Starting gyroscope-based position tracking...");

    vTaskDelay(pdMS_TO_TICKS(200));

    // Main measurement loop
    while (1) {
        if(calibration_flag) {
            ESP_LOGI(TAG, "Calibration triggered");

            // Send notification if Bluetooth is connected
            if(bluetooth_is_connected()) {
                const char *msg = "Calibration started";
                bluetooth_send_notification((const uint8_t *)msg, strlen(msg));
            }

            // Calibrate the gyroscope
            calibrate_gyroscope();

            // Save calibration to flash
            save_calibration_to_flash();

            // Send completion notification if Bluetooth is connected
            if(bluetooth_is_connected()) {
                char result_msg[128];
                snprintf(result_msg, sizeof(result_msg),
                         "Calibration complete: Range %.1f to %.1f deg",
                         min_angle, max_angle);
                bluetooth_send_notification((const uint8_t *)result_msg, strlen(result_msg));
            }

            calibration_flag = false;
            start_reading_data = false;

            // Reset measurement count when recalibrating
            measurement_count = 0;
        }

        if(read_measurements_flag) {
            ESP_LOGI(TAG, "Read measurements triggered via GPIO20");
            read_measurements_from_flash();
            read_measurements_flag = false;
        }

        if(erase_measurements_flag) {
            ESP_LOGI(TAG, "Erase measurements triggered via GPIO5");
            erase_measurements_from_flash();
            erase_measurements_flag = false;
        }

        if(download_data_flag) {
            ESP_LOGI(TAG, "CSV download triggered via GPIO6");
            download_measurements_csv();
            download_data_flag = false;
        }

        if(start_reading_data) {
            // Read gyroscope data
            mpu6500_scaled_data_t gyro;
            if (mpu6500_read_gyro(&mpu, &gyro) == ESP_OK) {
                // Remove offset and integrate angular velocity
                // CHANGED: Using X-axis instead of Y-axis based on diagnostic data
                float corrected_gyro_x = gyro.x - gyro_offset_x;
                float angle_delta = corrected_gyro_x * GYRO_SAMPLE_PERIOD_S;
                float unclamped_angle = current_angle + angle_delta;
                current_angle += angle_delta;

                // Clamp to calibrated range to prevent drift
                bool reversed = (min_angle > max_angle);
                float actual_min = reversed ? max_angle : min_angle;
                float actual_max = reversed ? min_angle : max_angle;

                if (current_angle < actual_min) current_angle = actual_min;
                if (current_angle > actual_max) current_angle = actual_max;

                // Convert to percentage
                uint8_t percentage = convert_to_percentage(current_angle);

                ESP_LOGI(TAG, "Gyro: %.2f deg/s, Delta: %.2f deg, Unclamped: %.2f, Angle: %.2f deg (range: %.2f to %.2f), %%: %d%%",
                         corrected_gyro_x, angle_delta, unclamped_angle, current_angle,
                         actual_min, actual_max, percentage);

                // Save measurement to flash
                save_measurement(current_angle, percentage);
            } else {
                ESP_LOGE(TAG, "Failed to read gyroscope data");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(GYRO_SAMPLE_PERIOD_MS));
    }
}
