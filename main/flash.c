#include "flash.h"
#include "config.h"
#include <string.h>
#include <stdlib.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FLASH";
static spi_device_handle_t spi;

// Static helper function prototypes
static esp_err_t flash_send_command(uint8_t cmd);
static esp_err_t flash_write_enable(void);
static uint8_t flash_read_status_register(void);
static void flash_cs_low(void);
static void flash_cs_high(void);

// Chip select control
static void flash_cs_low(void) {
    gpio_set_level(PIN_NUM_CS, 0);
}

static void flash_cs_high(void) {
    gpio_set_level(PIN_NUM_CS, 1);
}

// Send a single command
static esp_err_t flash_send_command(uint8_t cmd) {
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &cmd;
    
    flash_cs_low();
    esp_err_t ret = spi_device_transmit(spi, &t);
    flash_cs_high();
    
    return ret;
}

// Write enable
static esp_err_t flash_write_enable(void) {
    return flash_send_command(FLASH_CMD_WRITE_ENABLE);
}

// Read status register
static uint8_t flash_read_status_register(void) {
    spi_transaction_t t = {0};
    uint8_t tx_buf[2] = {FLASH_CMD_READ_STATUS_REG1, 0xFF};
    uint8_t rx_buf[2] = {0};
    
    t.length = 16;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    
    flash_cs_low();
    spi_device_transmit(spi, &t);
    flash_cs_high();
    
    return rx_buf[1];
}

// Initialize flash
esp_err_t flash_init(void) {
    // Configure CS pin
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_conf);
    flash_cs_high();
    
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
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }
    
    // SPI device config
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10*1000*1000,  // 10 MHz
        .mode = 0,
        .spics_io_num = -1,  // Manual CS
        .queue_size = 7,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }
    
    // Wait for flash to power up
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Read and verify manufacturer ID
    uint8_t mfg_id = flash_get_manufacturer_id();
    ESP_LOGI(TAG, "Flash Manufacturer ID: 0x%02X", mfg_id);
    
    if (mfg_id == 0x00 || mfg_id == 0xFF) {
        ESP_LOGE(TAG, "Invalid manufacturer ID! Check SPI connections.");
        return ESP_FAIL;
    }
    
    // Reset sequence (some chips need this)
    flash_send_command(FLASH_CMD_ENABLE_RESET);
    vTaskDelay(pdMS_TO_TICKS(1));
    flash_send_command(FLASH_CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "Flash initialized successfully");
    return ESP_OK;
}

// Write data to flash
esp_err_t flash_write(uint32_t address, const uint8_t *data, size_t len) {
    esp_err_t ret;
    size_t offset = 0;
    
    while (offset < len) {
        size_t chunk_size = len - offset;
        if (chunk_size > FLASH_PAGE_SIZE) {
            chunk_size = FLASH_PAGE_SIZE;
        }
        
        // Ensure we don't cross page boundary
        size_t page_offset = (address + offset) % FLASH_PAGE_SIZE;
        if (page_offset + chunk_size > FLASH_PAGE_SIZE) {
            chunk_size = FLASH_PAGE_SIZE - page_offset;
        }
        
        // Write enable
        ret = flash_write_enable();
        if (ret != ESP_OK) return ret;
        
        // Prepare write command
        spi_transaction_t t = {0};
        uint8_t *tx_buf = malloc(4 + chunk_size);
        if (!tx_buf) return ESP_ERR_NO_MEM;
        
        tx_buf[0] = FLASH_CMD_PAGE_PROGRAM;
        tx_buf[1] = ((address + offset) >> 16) & 0xFF;
        tx_buf[2] = ((address + offset) >> 8) & 0xFF;
        tx_buf[3] = (address + offset) & 0xFF;
        memcpy(&tx_buf[4], data + offset, chunk_size);
        
        t.length = (4 + chunk_size) * 8;
        t.tx_buffer = tx_buf;
        
        flash_cs_low();
        ret = spi_device_transmit(spi, &t);
        flash_cs_high();
        
        free(tx_buf);
        if (ret != ESP_OK) return ret;
        
        // Wait for write to complete
        flash_wait_busy();
        
        offset += chunk_size;
    }
    
    return ESP_OK;
}

// Read data from flash
// Saves the value to the array data that it is being pointed too
esp_err_t flash_read(uint32_t address, uint8_t *data, size_t len) {
    spi_transaction_t t = {0};
    uint8_t *tx_buf = malloc(4 + len);
    uint8_t *rx_buf = malloc(4 + len);
    
    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }
    
    tx_buf[0] = FLASH_CMD_READ_DATA;
    tx_buf[1] = (address >> 16) & 0xFF;
    tx_buf[2] = (address >> 8) & 0xFF;
    tx_buf[3] = address & 0xFF;
    memset(&tx_buf[4], 0xFF, len);
    
    t.length = (4 + len) * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    
    flash_cs_low();
    esp_err_t ret = spi_device_transmit(spi, &t);
    flash_cs_high();
    
    if (ret == ESP_OK) {
        memcpy(data, &rx_buf[4], len);
    }
    
    free(tx_buf);
    free(rx_buf);
    
    return ret;
}

// Erase sector (4KB)
esp_err_t flash_erase_sector(uint32_t address) {
    esp_err_t ret = flash_write_enable();
    if (ret != ESP_OK) return ret;
    
    spi_transaction_t t = {0};
    uint8_t tx_buf[4] = {
        FLASH_CMD_SECTOR_ERASE_4KB,
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        address & 0xFF
    };
    
    t.length = 32;
    t.tx_buffer = tx_buf;
    
    flash_cs_low();
    ret = spi_device_transmit(spi, &t);
    flash_cs_high();
    
    if (ret == ESP_OK) {
        flash_wait_busy();
    }
    
    return ret;
}

// Erase 32KB block
esp_err_t flash_erase_block_32k(uint32_t address) {
    esp_err_t ret = flash_write_enable();
    if (ret != ESP_OK) return ret;
    
    spi_transaction_t t = {0};
    uint8_t tx_buf[4] = {
        FLASH_CMD_BLOCK_ERASE_32KB,
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        address & 0xFF
    };
    
    t.length = 32;
    t.tx_buffer = tx_buf;
    
    flash_cs_low();
    ret = spi_device_transmit(spi, &t);
    flash_cs_high();
    
    if (ret == ESP_OK) {
        flash_wait_busy();
    }
    
    return ret;
}

// Erase 64KB block
esp_err_t flash_erase_block_64k(uint32_t address) {
    esp_err_t ret = flash_write_enable();
    if (ret != ESP_OK) return ret;
    
    spi_transaction_t t = {0};
    uint8_t tx_buf[4] = {
        FLASH_CMD_BLOCK_ERASE_64KB,
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        address & 0xFF
    };
    
    t.length = 32;
    t.tx_buffer = tx_buf;
    
    flash_cs_low();
    ret = spi_device_transmit(spi, &t);
    flash_cs_high();
    
    if (ret == ESP_OK) {
        flash_wait_busy();
    }
    
    return ret;
}

// Erase entire chip
esp_err_t flash_erase_chip(void) {
    esp_err_t ret = flash_write_enable();
    if (ret != ESP_OK) return ret;
    
    ret = flash_send_command(FLASH_CMD_CHIP_ERASE);
    if (ret == ESP_OK) {
        flash_wait_busy();
    }
    
    return ret;
}

// Test sequence:
void test_read_write(void){
    uint8_t test_write[4] = {0x12, 0x34, 0x56, 0x78};
    uint8_t test_read[4];

    flash_erase_sector(0x1000);              // MUST ERASE FIRST!
    flash_write(0x1000, test_write, 4);
    flash_read(0x1000, test_read, 4);

    ESP_LOGI(TAG, "Read: %02X %02X %02X %02X", 
            test_read[0], test_read[1], test_read[2], test_read[3]);
    // Should show: 12 34 56 78
}    

// Get manufacturer ID
uint8_t flash_get_manufacturer_id(void) {
    spi_transaction_t t = {0};
    uint8_t tx_buf[4] = {FLASH_CMD_READ_JEDEC_ID, 0xFF, 0xFF, 0xFF};
    uint8_t rx_buf[4] = {0};
    
    t.length = 32;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    
    flash_cs_low();
    spi_device_transmit(spi, &t);
    flash_cs_high();
    
    return rx_buf[1];  // Manufacturer ID is in byte 2
}

// Check if flash is busy
bool flash_is_busy(void) {
    return (flash_read_status_register() & FLASH_STATUS_BUSY) != 0;
}

// Wait for flash to complete operation
void flash_wait_busy(void) {
    while (flash_is_busy()) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Power down flash
void flash_power_down(void) {
    flash_send_command(FLASH_CMD_POWER_DOWN);
}

// Wake up flash
void flash_wake_up(void) {
    flash_send_command(FLASH_CMD_RELEASE_POWER_DOWN);
    vTaskDelay(pdMS_TO_TICKS(1));  // Typical wake-up time
}