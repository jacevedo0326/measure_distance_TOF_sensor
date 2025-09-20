#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

// Flash Commands
#define FLASH_CMD_WRITE_ENABLE          0x06
#define FLASH_CMD_WRITE_DISABLE         0x04
#define FLASH_CMD_READ_STATUS_REG1      0x05
#define FLASH_CMD_READ_STATUS_REG2      0x35
#define FLASH_CMD_WRITE_STATUS_REG      0x01
#define FLASH_CMD_PAGE_PROGRAM          0x02
#define FLASH_CMD_SECTOR_ERASE_4KB      0x20
#define FLASH_CMD_BLOCK_ERASE_32KB      0x52
#define FLASH_CMD_BLOCK_ERASE_64KB      0xD8
#define FLASH_CMD_CHIP_ERASE            0xC7
#define FLASH_CMD_ERASE_SUSPEND         0x75
#define FLASH_CMD_ERASE_RESUME          0x7A
#define FLASH_CMD_POWER_DOWN            0xB9
#define FLASH_CMD_READ_DATA             0x03
#define FLASH_CMD_FAST_READ             0x0B
#define FLASH_CMD_RELEASE_POWER_DOWN    0xAB
#define FLASH_CMD_DEVICE_ID             0xAB
#define FLASH_CMD_READ_MANUFACTURER_ID  0x90
#define FLASH_CMD_READ_JEDEC_ID         0x9F
#define FLASH_CMD_READ_UNIQUE_ID        0x4B
#define FLASH_CMD_READ_SFDP             0x5A
#define FLASH_CMD_ERASE_SECURITY_REG    0x44
#define FLASH_CMD_PROGRAM_SECURITY_REG  0x42
#define FLASH_CMD_READ_SECURITY_REG     0x48
#define FLASH_CMD_ENABLE_QPI            0x38
#define FLASH_CMD_ENABLE_RESET          0x66
#define FLASH_CMD_RESET                 0x99

// Status Register Bits
#define FLASH_STATUS_BUSY               0x01
#define FLASH_STATUS_WEL                0x02
#define FLASH_STATUS_BP0                0x04
#define FLASH_STATUS_BP1                0x08
#define FLASH_STATUS_BP2                0x10
#define FLASH_STATUS_TB                 0x20
#define FLASH_STATUS_SEC                0x40
#define FLASH_STATUS_SRP0               0x80

// Flash Parameters
#define FLASH_PAGE_SIZE                 256
#define FLASH_SECTOR_SIZE               4096
#define FLASH_BLOCK_32KB_SIZE           32768
#define FLASH_BLOCK_64KB_SIZE           65536

// Manufacturer IDs
#define FLASH_MFG_WINBOND               0xEF
#define FLASH_MFG_MACRONIX              0xC2
#define FLASH_MFG_MICRON                0x20
#define FLASH_MFG_SPANSION              0x01
#define FLASH_MFG_GIGADEVICE            0xC8

// Public API Functions
esp_err_t flash_init(void);
esp_err_t flash_write(uint32_t address, const uint8_t *data, size_t len);
esp_err_t flash_read(uint32_t address, uint8_t *data, size_t len);
esp_err_t flash_erase_sector(uint32_t address);
esp_err_t flash_erase_block_32k(uint32_t address);
esp_err_t flash_erase_block_64k(uint32_t address);
esp_err_t flash_erase_chip(void);
uint8_t flash_get_manufacturer_id(void);
void test_read_write(void);
bool flash_is_busy(void);
void flash_wait_busy(void);
void flash_power_down(void);
void flash_wake_up(void);

#endif // FLASH_H