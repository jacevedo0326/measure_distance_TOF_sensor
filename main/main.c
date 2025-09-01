#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "vl53l0x.h"

//defines
#define device_interrupt_pin 3
volatile bool data_ready_flag = false;
TaskHandle_t check_address_handle = NULL;
vl53l0x_t *sensor = NULL;  // Global sensor variable

//function inits
static void IRAM_ATTR data_ready(void* arg);

// The ESP32-VL53L0X library will handle I2C initialization

void test_i2c_scan(void) {
    printf("Scanning I2C bus...\n");
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("Found I2C device at: 0x%02x\n", addr);
        }
    }
    printf("I2C scan complete\n");
}

void sensor_init(void){
    // Initialize VL53L0X sensor using ESP32 library
    sensor = vl53l0x_config(I2C_NUM_0, GPIO_NUM_9, GPIO_NUM_8, -1, 0x29, false);
    if (sensor == NULL) {
        printf("Failed to configure VL53L0X sensor\n");
        return;
    }
    
    if (vl53l0x_init(sensor) != ESP_OK) {
        printf("Failed to initialize VL53L0X sensor\n");
        return;
    }
    
    // Start continuous measurements with 100ms period
    vl53l0x_startContinuous(sensor, 100);
    printf("VL53L0X sensor initialized successfully\n");
}

void intr_init(void){
    gpio_config_t intr_config = {
        .pin_bit_mask = (1ULL << device_interrupt_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&intr_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(device_interrupt_pin, data_ready, NULL);
}

static void IRAM_ATTR data_ready(void* arg){
    data_ready_flag = true;
}

void app_main(void)
{
    // First scan I2C bus to see what's connected
    test_i2c_scan();
    
    // Initialize sensor
    sensor_init();
    intr_init();
    
    printf("Starting VL53L0X distance measurements...\n");
    
    while(1){
        if(data_ready_flag == true){
            uint16_t distance = vl53l0x_readRangeContinuousMillimeters(sensor);
            printf("Distance: %d mm\n", distance);
            data_ready_flag = false;  // Clear the flag
        }else{
            printf("Data not ready");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Small delay to prevent busy waiting
    }
}