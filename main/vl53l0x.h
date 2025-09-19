#ifndef VL53L0X_H
#define VL53L0X_H

#include "esp_err.h"
#include <stdint.h>

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

// Public function declarations - sensor-specific only
void vl53l0x_init_simple(void);
uint16_t read_range_continuous(void);
void start_continuous(uint32_t period_ms);

#endif