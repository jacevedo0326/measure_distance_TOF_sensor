#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// BLE device configuration
#define BLE_DEVICE_NAME         "ESP32-C3-Measure"

// Callback type for received BLE data
typedef void (*bluetooth_rx_callback_t)(const uint8_t *data, uint16_t len);

/**
 * @brief Initialize Bluetooth (NimBLE) stack and start advertising
 *
 * This function:
 * - Initializes NVS (required for BLE)
 * - Configures NimBLE stack
 * - Creates GATT service and characteristics
 * - Starts BLE advertising
 *
 * @param rx_callback Callback function to handle received data (can be NULL)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bluetooth_init(bluetooth_rx_callback_t rx_callback);

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
