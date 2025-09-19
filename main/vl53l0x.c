#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "vl53l0x.h"

static const char *TAG = "VL53L0X";

// Private I2C communication functions
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

// Public functions

// VL53L0X initialization
void vl53l0x_init_simple(void) {
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

// Read continuous measurement
uint16_t read_range_continuous(void) {
    uint16_t range = read_reg16(RESULT_RANGE_STATUS + 10);
    return range;
}

// Start continuous measurements
void start_continuous(uint32_t period_ms) {
    write_reg(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    write_reg(SYSRANGE_START, 0x02);
}