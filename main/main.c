#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/spi_master.h"

// VL53L0X Register addresses
#define SYSRANGE_START                          0x00
#define SYSTEM_SEQUENCE_CONFIG                  0x01
#define SYSTEM_INTERMEASUREMENT_PERIOD          0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define VL53L0X_ADDR                            0x29
#define GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define SYSTEM_INTERRUPT_CLEAR                  0x0B
#define RESULT_INTERRUPT_STATUS                 0x13
#define RESULT_RANGE_STATUS                     0x14
#define IDENTIFICATION_MODEL_ID                 0xC0
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV      0x89
#define MSRC_CONFIG_CONTROL                     0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44

// SPI INIT - Using available pins on ESP32-C3
#define PIN_NUM_MISO 2   // GPIO2 (A2 pin)
#define PIN_NUM_MOSI 3   // GPIO3 (A3 pin)  
#define PIN_NUM_CLK  4   // GPIO4 (A4 pin)
#define PIN_NUM_CS   10  // GPIO10

static const char *TAG = "VL53L0X";

// Global Variables
volatile uint16_t lower_bound = 0;
volatile uint16_t upper_bound = 0;
volatile uint16_t true_upper_bound = 0;
spi_device_handle_t spi;

// I2C communication functions
static esp_err_t write_reg(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t write_reg16(uint8_t reg, uint16_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, (data >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, data & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return data;
}

static uint16_t read_reg16(uint8_t reg) {
    uint16_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_READ, true);
    uint8_t data_h, data_l;
    i2c_master_read_byte(cmd, &data_h, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data_l, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    data = (data_h << 8) | data_l;
    return data;
}

// VL53L0X initialization
static void vl53l0x_init_simple(void) {
    uint8_t model_id = read_reg(IDENTIFICATION_MODEL_ID);
    ESP_LOGI(TAG, "Model ID: 0x%02X (expected 0xEE)", model_id);
    
    uint8_t vhv_config = read_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
    write_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, vhv_config | 0x01);
    
    write_reg(0x88, 0x00);
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xFF);
    write_reg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0x0020);
    write_reg(SYSTEM_INTERMEASUREMENT_PERIOD, 0x00);
    
    ESP_LOGI(TAG, "Simple initialization complete");
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

// Start continuous measurements
static void start_continuous(uint32_t period_ms) {
    write_reg(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    write_reg(SYSRANGE_START, 0x02);
}

// Read continuous measurement
static uint16_t read_range_continuous(void) {
    uint16_t range = read_reg16(RESULT_RANGE_STATUS + 10);
    return range;
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

// Flash helper functions
static void flash_write_enable(void) {
    spi_transaction_t t = {0};
    uint8_t cmd = 0x06;
    t.length = 8;
    t.tx_buffer = &cmd;
    
    gpio_set_level(PIN_NUM_CS, 0);
    spi_device_transmit(spi, &t);
    gpio_set_level(PIN_NUM_CS, 1);
}

static uint8_t flash_read_status(void) {
    spi_transaction_t t = {0};
    uint8_t tx_buf[2] = {0x05, 0xFF};
    uint8_t rx_buf[2] = {0};
    
    t.length = 16;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    
    gpio_set_level(PIN_NUM_CS, 0);
    spi_device_transmit(spi, &t);
    gpio_set_level(PIN_NUM_CS, 1);
    
    return rx_buf[1];
}

static void flash_wait_busy(void) {
    uint8_t status;
    do {
        status = flash_read_status();
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (status & 0x01);
}

static void flash_write_data(uint32_t address, uint8_t *data, size_t len) {
    flash_write_enable();
    
    spi_transaction_t t = {0};
    uint8_t *tx_buf = malloc(4 + len);
    
    tx_buf[0] = 0x02;  // Page Program command
    tx_buf[1] = (address >> 16) & 0xFF;
    tx_buf[2] = (address >> 8) & 0xFF;
    tx_buf[3] = address & 0xFF;
    memcpy(&tx_buf[4], data, len);
    
    t.length = (4 + len) * 8;
    t.tx_buffer = tx_buf;
    
    gpio_set_level(PIN_NUM_CS, 0);
    spi_device_transmit(spi, &t);
    gpio_set_level(PIN_NUM_CS, 1);
    
    free(tx_buf);
    flash_wait_busy();
}

static void flash_read_data(uint32_t address, uint8_t *data, size_t len) {
    spi_transaction_t t = {0};
    uint8_t *tx_buf = malloc(4 + len);
    uint8_t *rx_buf = malloc(4 + len);
    
    tx_buf[0] = 0x03;  // Read Data command
    tx_buf[1] = (address >> 16) & 0xFF;
    tx_buf[2] = (address >> 8) & 0xFF;
    tx_buf[3] = address & 0xFF;
    memset(&tx_buf[4], 0xFF, len);
    
    t.length = (4 + len) * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    
    gpio_set_level(PIN_NUM_CS, 0);
    spi_device_transmit(spi, &t);
    gpio_set_level(PIN_NUM_CS, 1);
    
    memcpy(data, &rx_buf[4], len);
    
    free(tx_buf);
    free(rx_buf);
}

// Initialize flash
static esp_err_t flash_init(void) {
    // Configure CS pin
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_conf);
    gpio_set_level(PIN_NUM_CS, 1);
    
    // SPI bus config
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;
    
    // SPI device config
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10*1000*1000,  // 10 MHz
        .mode = 0,
        .spics_io_num = -1,  // Manual CS
        .queue_size = 7,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) return ret;
    
    // Wait for flash to power up
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Read JEDEC ID to verify flash
    uint8_t tx_buf[4] = {0x9F, 0xFF, 0xFF, 0xFF};
    uint8_t rx_buf[4] = {0};
    
    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    
    gpio_set_level(PIN_NUM_CS, 0);
    spi_device_transmit(spi, &t);
    gpio_set_level(PIN_NUM_CS, 1);
    
    return ESP_OK;
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
    flash_write_data(0x000000, calib_data, 4);
    
    // Read back to verify
    uint8_t verify_data[4];
    flash_read_data(0x000000, verify_data, 4);
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