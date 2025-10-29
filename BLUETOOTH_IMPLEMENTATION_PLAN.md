# Bluetooth Implementation Plan for ESP32-C3 Supermini

## Overview
We'll use **NimBLE** (the recommended lightweight Bluetooth stack for ESP32-C3) to create a BLE SPP (Serial Port Profile) service that mirrors your hardware button functionality over Bluetooth.

## Implementation Plan

### Phase 1: Basic Bluetooth "Hello World"
**Goal**: Get basic BLE communication working and verify the stack

1. ✅ **Verify ESP32-C3 Bluetooth driver availability**
   - Confirmed NimBLE stack is available in ESP-IDF v5.5
   - ESP32-C3 supports Bluetooth 5.0

2. **Create separate bluetooth.c and bluetooth.h files**
   - Place in `main/` directory alongside flash.c and vl53l0x.c
   - Follow same modular architecture pattern

3. **Implement basic NimBLE initialization**
   - Initialize NVS (required for BLE)
   - Initialize NimBLE stack
   - Create simple GATT service with one characteristic
   - Implement "Hello World" response when written to

4. **Update main/CMakeLists.txt**
   - Add bluetooth.c to SRCS list
   - Add required NimBLE components

5. **Enable Bluetooth in sdkconfig**
   - Enable CONFIG_BT_ENABLED
   - Enable CONFIG_BT_NIMBLE_ENABLED
   - Configure NimBLE parameters

6. **Build and test basic functionality**
   - Verify device advertises as "ESP32-C3-Measure"
   - Connect from phone/computer using BLE scanner app
   - Test basic read/write to characteristic
   - Verify "Hello World" response

### Phase 2: Command Protocol Design
**Goal**: Define communication protocol for all device functions

Define simple single-byte commands that mirror GPIO buttons:
- `0x01` - Start/Stop measurements (mirrors GPIO1)
- `0x02` - Calibrate (mirrors GPIO20)
- `0x03` - Read measurements (mirrors GPIO21)
- `0x04` - Erase measurements (mirrors GPIO5)
- `0x05` - Download CSV (mirrors GPIO6)

Response format:
- Success: `0xAA` + command byte
- Failure: `0xFF` + command byte + error code

### Phase 3: Integration with Existing System
**Goal**: Make Bluetooth commands trigger same actions as hardware buttons

- Bluetooth write handler sets the same global flags:
  - `start_reading_data`
  - `calibration_flag`
  - `read_measurements_flag`
  - `erase_measurements_flag`
  - `download_data_flag`

- Main loop processes flags identically regardless of source (GPIO or BLE)
- Add BLE notifications to send responses back to phone
- Implement CSV data streaming over BLE for download command

## Architecture Benefits

- **Modular Design**: Bluetooth code completely isolated in bluetooth.c/h
- **Flag-Based**: Uses same flag approach as existing GPIO interrupts
- **Zero Core Changes**: No modifications to measurement/calibration logic
- **Dual Control**: Hardware buttons continue working alongside Bluetooth
- **Consistent Pattern**: Follows same separation as flash.c and vl53l0x.c

## Key Files

### New Files
- `main/bluetooth.c` - All BLE initialization and handlers
- `main/bluetooth.h` - BLE API and command definitions

### Modified Files
- `main/CMakeLists.txt` - Add bluetooth.c to build
- `main/main.c` - Initialize Bluetooth, handle BLE flags
- `sdkconfig` - Enable NimBLE components (auto-generated)

## Testing Strategy

### Phase 1 Testing
1. Use nRF Connect app (Android/iOS) to scan for device
2. Verify device name appears
3. Connect and discover services
4. Write to characteristic and verify "Hello World" response

### Phase 2 Testing
1. Send each command byte (0x01-0x05)
2. Verify appropriate flags are set in main loop
3. Confirm actions execute (calibration, measurement, etc.)
4. Verify response notifications received

### Phase 3 Testing
1. Test simultaneous GPIO button and BLE commands
2. Verify CSV download over BLE
3. Test connection/disconnection stability
4. Measure power consumption impact

## Implementation Notes

- NimBLE is more memory-efficient than Bluedroid for ESP32-C3
- BLE characteristic handles 20 bytes per packet (can be negotiated higher with MTU)
- CSV download may need chunking for large datasets
- Consider adding authentication/pairing for production use
