# Build Instructions for Bluetooth Support

## Problem
The build is failing because the NimBLE Bluetooth headers aren't being found. This is because the `sdkconfig` needs to be regenerated with the new Bluetooth settings from `sdkconfig.defaults`.

## Solution - Clean Build

Run these commands in order:

```bash
# Step 1: Clean the existing build completely
idf.py fullclean

# Step 2: Reconfigure the project (this will read sdkconfig.defaults)
idf.py reconfigure

# Step 3: Build the project
idf.py build
```

## What This Does

1. **fullclean** - Removes the entire `build/` directory and `sdkconfig` file
2. **reconfigure** - Regenerates `sdkconfig` from `sdkconfig.defaults`, enabling:
   - Bluetooth controller
   - NimBLE stack (not Bluedroid)
   - BLE peripheral role
   - Device name: "ESP32-C3-Measure"
3. **build** - Compiles with Bluetooth support

## Alternative: Manual Configuration

If `fullclean` doesn't work, you can manually:

```bash
# Delete build folder and sdkconfig
rm -rf build
rm sdkconfig

# Then build (will use sdkconfig.defaults automatically)
idf.py build
```

## After Successful Build

Flash and monitor:
```bash
idf.py -p COM# flash monitor
```

Replace `COM#` with your actual port (e.g., COM3, COM4, etc.)

## Expected Output

You should see log messages like:
```
I (XXX) BLUETOOTH: Initializing Bluetooth...
I (XXX) BLUETOOTH: Initializing NimBLE...
I (XXX) BLUETOOTH: BLE stack synced
I (XXX) BLUETOOTH: Advertising started as 'ESP32-C3-Measure'
I (XXX) BLUETOOTH: Bluetooth initialized successfully
I (XXX) BLUETOOTH: Device name: ESP32-C3-Measure
```

## Testing with Phone

1. Download "nRF Connect" app (Android/iOS)
2. Scan for Bluetooth devices
3. Look for "ESP32-C3-Measure"
4. Connect to it
5. Discover services and characteristics
6. Write commands to the RX characteristic:
   - `0x01` - Start/Stop measurements
   - `0x02` - Calibrate
   - `0x03` - Read measurements
   - `0x04` - Erase measurements
   - `0x05` - Download CSV

You should receive "Hello World!" back on any write, and see log messages showing the command was received.
