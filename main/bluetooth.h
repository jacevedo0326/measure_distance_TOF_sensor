#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Bluetooth command definitions (mirror GPIO button functions)
#define BLE_CMD_START_STOP      0x01  // Start/Stop measurements (GPIO1)
#define BLE_CMD_CALIBRATE       0x02  // Start calibration (GPIO20)
#define BLE_CMD_READ_MEAS       0x03  // Read measurements (GPIO21)
#define BLE_CMD_ERASE_MEAS      0x04  // Erase measurements (GPIO5)
#define BLE_CMD_DOWNLOAD_CSV    0x05  // Download CSV data (GPIO6)

// Response codes
#define BLE_RESP_SUCCESS        0xAA
#define BLE_RESP_ERROR          0xFF

// BLE device configuration
#define BLE_DEVICE_NAME         "ESP32-C3-Measure"

// External flags that Bluetooth will set (same as GPIO interrupts)
extern volatile bool start_reading_data;
extern volatile bool calibration_flag;
extern volatile bool read_measurements_flag;
extern volatile bool erase_measurements_flag;
extern volatile bool download_data_flag;

/**
 * @brief Initialize Bluetooth (NimBLE) stack and start advertising
 *
 * This function:
 * - Initializes NVS (required for BLE)
 * - Configures NimBLE stack
 * - Creates GATT service and characteristics
 * - Starts BLE advertising
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bluetooth_init(void);

/**
 * @brief Send a notification to connected BLE client
 *
 * @param data Pointer to data buffer to send
 * @param len Length of data to send
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bluetooth_send_notification(const uint8_t *data, uint16_t len);

/**
 * @brief Check if a BLE client is currently connected
 *
 * @return true if connected, false otherwise
 */
bool bluetooth_is_connected(void);

/**
 * @brief Test Bluetooth connection by sending "Hello World" every 10 seconds
 *
 * This function runs in a loop and sends "Hello World" notifications
 * every 10 seconds to verify the Bluetooth connection is working.
 */
void test_bluetooth_connection(void);

#endif // BLUETOOTH_H
