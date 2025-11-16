# MPU6500 Pedal Position Tracker - Control Reference

## GPIO Pin Assignments

| GPIO | Function | Description |
|------|----------|-------------|
| GPIO8 | I2C SDA | MPU6500 Data Line |
| GPIO9 | I2C SCL | MPU6500 Clock Line |
| GPIO20 | Calibrate | Start gyroscope calibration |
| GPIO21 | Erase | Hold for 3 seconds to erase measurements |
| GPIO6 | Start/Stop | Toggle measurement recording |
| GPIO5 | Download CSV | Download all measurements as CSV |

## Bluetooth Commands

Send these byte values to the BLE characteristic to control the device:

| Command | Hex Value | Function |
|---------|-----------|----------|
| Start/Stop Measurements | `0x01` | Toggle measurement recording |
| Calibrate | `0x02` | Start gyroscope calibration |
| Download CSV Data | `0x03` | Download all measurements via serial |
| Erase Measurements | `0x04` | Erase all stored measurements |

## Usage Instructions

### First Time Setup
1. Press GPIO20 (or send `0x02` via BLE) to calibrate
2. Follow the on-screen prompts
3. Pedal will be calibrated and ready to use

### Recording Measurements
1. Press GPIO6 (or send `0x01` via BLE) to start recording
2. Press GPIO6 again to stop recording
3. Measurements are automatically saved to flash

### Downloading Data
1. Press GPIO5 (or send `0x03` via BLE)
2. CSV data will be printed to serial console
3. Look for `===CSV_START===` and `===CSV_END===` markers

### Erasing All Measurements
**Option 1:** Hold GPIO21 for 3 seconds
**Option 2:** Send `0x04` via Bluetooth

### Recalibration
1. Press GPIO20
2. OR send `0x02` via Bluetooth
3. All measurements will be erased automatically
4. Follow calibration prompts

## Sample Rates

- **Gyroscope Sampling:** 20 Hz (50ms intervals)
- **Measurement Storage:** Every sample when recording is active
- **Calibration:** 100 samples over 10 seconds

## Calibration Requirements

- Minimum range: 5 degrees
- Typical range: 70-90 degrees for pedal motion
- Gyro offset calibration: 100 samples while stationary
