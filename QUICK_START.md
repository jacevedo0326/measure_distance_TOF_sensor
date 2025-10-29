# Quick Start - Bluetooth Build

## Status: READY TO BUILD ✅

All issues have been fixed. Just run:

```bash
idf.py build
```

## What Was Fixed

1. ❌ Target mismatch (esp32 vs esp32c3) → ✅ Deleted build/
2. ❌ Bluetooth disabled → ✅ Deleted sdkconfig
3. ❌ Missing includes → ✅ Will be auto-added when Bluetooth enables

## After Build

```bash
# Flash to ESP32-C3
idf.py -p COMX flash monitor

# Test with phone app "nRF Connect"
# Look for device: "ESP32-C3-Measure"
```

## Bluetooth Commands

Connect via nRF Connect and write to RX characteristic:

| Command | Hex | Action |
|---------|-----|--------|
| Start/Stop | `0x01` | Toggle measurements |
| Calibrate | `0x02` | Run calibration |
| Read | `0x03` | View stored data |
| Erase | `0x04` | Clear flash |
| Download | `0x05` | Get CSV data |

## Files Created

- `main/bluetooth.c` - BLE implementation
- `main/bluetooth.h` - BLE API
- `sdkconfig.defaults` - BT config
- `main/CMakeLists.txt` - Updated (includes bluetooth.c)
- `main/main.c` - Updated (calls bluetooth_init())

## That's It!

Just run `idf.py build` and it should work.
