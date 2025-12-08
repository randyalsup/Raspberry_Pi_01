# ESP32 Firmware Refactoring Summary

## What Changed

The ESP32 firmware has been **simplified to be pure UART ↔ ESP-NOW bridges**. All sensor reading and control logic has been moved to the Raspberry Pis where it belongs.

## Old Architecture (Incorrect)

```
Robot ESP32:
- ❌ Read ADS1115 (battery/current)
- ❌ Generate telemetry
- ❌ OTA updates
- ✅ ESP-NOW communication

Remote ESP32:
- ❌ Parse status messages
- ❌ Display formatting
- ✅ ESP-NOW communication
```

## New Architecture (Correct)

```
Robot Side:
┌─────────────────┐
│ Raspberry Pi 5  │ ← Does ALL the work
│  - ADS1115      │   - Battery monitoring
│  - Motors       │   - Motor control
│  - Sensors      │   - Sensor reading
│  - Logic        │   - Control logic
└────────┬────────┘
         │ UART (115200 baud, framed protocol)
         ↓
┌─────────────────┐
│ ESP32 (RO)      │ ← Simple transparent bridge
│  - UART RX/TX   │   - Forward packets
│  - ESP-NOW      │   - No logic
└─────────────────┘

Remote Side:
┌─────────────────┐
│ Pi Zero 2W      │ ← Does ALL the work
│  - Touch UI     │   - Display graphics
│  - User input   │   - Process touch
│  - Logic        │   - UI state machine
└────────┬────────┘
         │ UART (115200 baud, framed protocol)
         ↓
┌─────────────────┐
│ ESP32 (RC)      │ ← Simple transparent bridge
│  - UART RX/TX   │   - Forward packets
│  - ESP-NOW      │   - No logic
└─────────────────┘
```

## File Changes

### Created

1. **`firmware/esp32/boards/robot/src/main_bridge.cpp`**
   - New simplified bridge firmware (135 lines vs 280 lines)
   - UART protocol implementation
   - ESP-NOW forwarding
   - Ready for UWB integration

2. **`firmware/esp32/boards/remote/src/main_bridge.cpp`**
   - Mirror of robot bridge
   - Same protocol, opposite direction

3. **`docs/protocol/PI_ESP32_UART.md`**
   - Complete protocol specification
   - Message types and formats
   - CRC-8 implementation
   - Python and C++ examples

4. **`firmware/esp32/boards/robot/src/main.cpp.backup`**
   - Backup of old firmware (for reference)

### Modified

1. **`firmware/esp32/boards/robot/platformio.ini`**
   - Removed Adafruit ADS1X15 library dependency
   - Removed OTA configuration (not needed)
   - Clean minimal config

2. **`firmware/esp32/boards/robot/include/pin_config.h`**
   - Already has UART pins defined (GPIO16/17)
   - Already has UWB pin allocations for future

## How to Use New Firmware

### Option 1: Replace Existing Files

```powershell
# Backup current
Move-Item firmware/esp32/boards/robot/src/main.cpp firmware/esp32/boards/robot/src/main_old.cpp
Move-Item firmware/esp32/boards/remote/src/main.cpp firmware/esp32/boards/remote/src/main_old.cpp

# Use new bridge firmware
Move-Item firmware/esp32/boards/robot/src/main_bridge.cpp firmware/esp32/boards/robot/src/main.cpp
Move-Item firmware/esp32/boards/remote/src/main_bridge.cpp firmware/esp32/boards/remote/src/main.cpp
```

### Option 2: Test Side-by-Side

Keep both files and modify platformio.ini to specify which to build:

```ini
[env:esp32dev]
build_src_filter = +<main_bridge.cpp> -<main.cpp>
```

## Benefits of New Architecture

✅ **Simpler ESP32 code** - 135 lines vs 280 lines  
✅ **Faster development** - Python on Pi is easier than C++ on ESP32  
✅ **More powerful** - Pi 5 can do complex processing  
✅ **Better separation** - Each device has clear role  
✅ **Easier debugging** - Can test Pi code independently  
✅ **More flexible** - Can update Pi code via SSH  
✅ **Ready for UWB** - Pin allocations already planned  

## Protocol Features

- **Framed packets**: SYNC (0xAA) + TYPE + LENGTH + PAYLOAD + CRC8 + END (0x55)
- **CRC-8 checksum**: Detects corruption
- **Multiple message types**: Commands, status, telemetry, UWB data
- **Bidirectional**: Both Pis can send/receive
- **Efficient**: 5-byte overhead per message
- **Reliable**: Error detection and recovery

## Next Steps

1. **Flash new ESP32 firmware** (both robot and remote)
2. **Create Python code for Pi 5** (ADS1115, motors, UART)
3. **Create Python code for Pi Zero 2W** (Touch UI, UART)
4. **Test UART communication** (echo test between Pi and ESP32)
5. **Test end-to-end** (Pi 5 → ESP32 RO → ESP32 RC → Pi Zero 2W)
6. **Add UWB support** (when DWM3000 modules arrive)

## Migration Checklist

- [x] Protocol designed
- [x] ESP32 bridge firmware written
- [x] Pin configs updated for UWB
- [x] Old firmware backed up
- [ ] Flash new firmware to both ESP32s
- [ ] Write Pi 5 Python code
- [ ] Write Pi Zero 2W Python code
- [ ] Test UART loopback
- [ ] Test end-to-end communication

## Testing the New Firmware

**Without Pi connected (standalone test):**

```cpp
// In main_bridge.cpp, add to loop():
if (Serial.available()) {
  uint8_t test[10] = {0xAA, 0x01, 0x02, 0x11, 0x22, 0x00, 0x55};
  Serial2.write(test, 7);  // Echo test packet
}
```

**With Pi connected:**

```python
# On Raspberry Pi
import serial
ser = serial.Serial('/dev/serial0', 115200)
test_packet = bytes([0xAA, 0x01, 0x02, 0x11, 0x22, 0x00, 0x55])
ser.write(test_packet)
response = ser.read(7)
print("Received:", response.hex())
```

## Rollback Plan

If new firmware doesn't work, restore from backup:

```powershell
Copy-Item firmware/esp32/boards/robot/src/main.cpp.backup firmware/esp32/boards/robot/src/main.cpp
platformio run --target upload
```

---

**Status:** Ready to flash and test  
**Date:** 2025-12-02
