# OTA (Over-The-Air) Update Setup for Robot ESP32

## Overview
The robot ESP32 supports firmware updates over WiFi using Arduino OTA when connected to the Raspberry Pi 5 at **192.168.1.113**.

## Initial Setup

### 1. Configure WiFi Credentials (First Time Only)
Edit `src/main.cpp` and set your WiFi credentials:

```cpp
#define WIFI_SSID "YourWiFiNetwork"     // Your WiFi network name
#define WIFI_PASSWORD "YourWiFiPassword" // Your WiFi password
```

### 2. Upload Initial Firmware via USB
```powershell
cd C:\Users\randy\OneDrive\Documents\PlatformIO\Projects\Raspberry_Pi_01\firmware\esp32\boards\robot
C:\Users\randy\AppData\Roaming\Python\Python312\Scripts\platformio.exe run --target upload
```

The ESP32 will:
- Connect to your WiFi network
- Print its IP address to serial monitor
- Enable OTA service on port 3232
- Continue normal ESP-NOW operation

### 3. Verify OTA is Ready
Monitor the serial output - you should see:
```
Connecting to WiFi for OTA... connected!
IP: 192.168.1.XXX
OTA ready
```

## Updating Firmware Over WiFi

### Method 1: From Windows Development Machine
Edit `platformio.ini` and uncomment the OTA section:

```ini
; OTA upload via Raspberry Pi 5
upload_protocol = espota
upload_port = 192.168.1.113  ; IP of the ESP32 or Pi 5 acting as bridge
upload_flags = 
    --port=3232
    --auth=robotota
```

Then upload:
```powershell
platformio run --target upload
```

### Method 2: From Raspberry Pi 5 (Recommended for robot)
If the ESP32 is connected to the Pi 5 via UART and shares the Pi's network:

```bash
# On the Pi 5
cd /path/to/firmware/esp32/boards/robot

# Option A: Using PlatformIO on Pi
pio run --target upload

# Option B: Using espota.py directly
~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
  -i 192.168.1.113 \
  -p 3232 \
  --auth=robotota \
  -f .pio/build/esp32dev/firmware.bin
```

## Network Configuration

### ESP32 Connection Modes

**Development Mode (OTA enabled):**
- WiFi: Connected to local network for OTA updates
- ESP-NOW: Active for robot ↔ remote communication
- Both protocols can coexist (WiFi + ESP-NOW on same channel)

**Production Mode (WiFi disabled):**
- Set `#define WIFI_SSID ""` to disable WiFi connection
- ESP-NOW only (lower power, no router dependency)
- OTA not available until WiFi re-enabled

### IP Address Assignment
The ESP32's IP is assigned by DHCP. To ensure consistent OTA access:

**Option 1: DHCP Reservation (recommended)**
Configure your router to always assign the same IP to the ESP32's MAC address.

**Option 2: Static IP**
Modify the WiFi setup in `main.cpp`:
```cpp
WiFi.config(
  IPAddress(192, 168, 1, 113),  // Static IP
  IPAddress(192, 168, 1, 1),    // Gateway
  IPAddress(255, 255, 255, 0)   // Subnet mask
);
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
```

## Security Notes

1. **Change default OTA password** before deploying:
   ```cpp
   #define OTA_PASSWORD "your_secure_password_here"
   ```

2. **OTA port 3232** must be accessible on your network
   - Firewall may need configuration
   - Not exposed to internet by default

3. **WiFi credentials** are stored in firmware
   - Use WPA2/WPA3 encrypted network
   - Consider using secrets management for production

## Troubleshooting

### "OTA failed" or device not found
- Verify ESP32 is connected to WiFi (check serial output)
- Ping the ESP32 IP: `ping 192.168.1.113`
- Check firewall allows port 3232
- Verify OTA password matches in both firmware and platformio.ini

### ESP-NOW stops working after enabling WiFi
- ESP-NOW and WiFi can coexist on the same channel
- If issues occur, ensure both use the same WiFi channel (usually channel 1)
- Check that `WiFi.mode(WIFI_STA)` is called before `esp_now_init()`

### High power consumption
- WiFi uses significantly more power than ESP-NOW alone
- For battery operation, disable WiFi in production:
  ```cpp
  #define WIFI_SSID ""  // Disables WiFi connection attempt
  ```
- Enable WiFi only when updates needed

## Production Deployment Workflow

1. **Development phase**: WiFi + OTA enabled for rapid iteration
2. **Testing phase**: Disable OTA, enable WiFi for monitoring
3. **Production phase**: WiFi disabled, ESP-NOW only for lowest power
4. **Update phase**: Re-enable WiFi temporarily via serial config or physical button

## Alternative: USB Serial Updates (No OTA)
If WiFi/OTA is not desired, always use USB serial upload:

```ini
; platformio.ini
upload_port = COM3
upload_speed = 115200
; Do NOT set upload_protocol = espota
```

This is simpler but requires physical access to the ESP32.
