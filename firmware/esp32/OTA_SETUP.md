# ESP32 OTA (Wireless) Setup & Connection Guide

## Step 1: Update WiFi Credentials

Edit both firmware files and replace with your WiFi network:

**Robot:** `firmware/esp32/boards/robot/src/main.cpp`
**Remote:** `firmware/esp32/boards/remote/src/main.cpp`

```cpp
const char* ssid = "YOUR_WIFI_SSID";      // Replace with your WiFi name
const char* password = "YOUR_WIFI_PASSWORD";  // Replace with your WiFi password
```

## Step 2: Physical Connections

### Robot: ESP32 (RO) ↔ Raspberry Pi 5

| ESP32 Pin | Pi 5 Pin | Wire Color | Function |
|-----------|----------|------------|----------|
| GPIO16 (RX2) | GPIO14 (TXD) | Yellow | Pi → ESP32 data |
| GPIO17 (TX2) | GPIO15 (RXD) | Orange | ESP32 → Pi data |
| GND | GND (Pin 6) | Black | Common ground |
| 5V (optional) | 5V (Pin 2) | Red | Power from Pi |

**Note:** You can power ESP32 via USB or from Pi's 5V pin (not both!)

### Remote: ESP32 (RC) ↔ Raspberry Pi Zero 2W

| ESP32 Pin | Pi Zero Pin | Wire Color | Function |
|-----------|-------------|------------|----------|
| GPIO16 (RX2) | GPIO14 (TXD) | Yellow | Pi → ESP32 data |
| GPIO17 (TX2) | GPIO15 (RXD) | Orange | ESP32 → Pi data |
| GND | GND (Pin 6) | Black | Common ground |
| 5V (optional) | 5V (Pin 2) | Red | Power from Pi |

## Step 3: Flash ESP32s with OTA Firmware (One-Time USB)

### Robot ESP32:
```powershell
cd firmware\esp32\boards\robot
C:\Users\randy\AppData\Roaming\Python\Python312\Scripts\platformio.exe run --target upload
```

### Remote ESP32:
```powershell
cd firmware\esp32\boards\remote
C:\Users\randy\AppData\Roaming\Python\Python312\Scripts\platformio.exe run --target upload
```

## Step 4: Verify WiFi Connection

Open serial monitor to see IP addresses:
```powershell
platformio device monitor --port COM3
```

You should see:
```
WiFi connected!
IP: 192.168.1.xxx
MAC: 00:70:07:E6:CF:7C
OTA ready
```

Note the IP addresses for each ESP32!

## Step 5: Future OTA Updates (Wireless!)

Once WiFi is configured, you can update firmware wirelessly:

### Robot ESP32 (replace with actual IP):
```powershell
cd firmware\esp32\boards\robot
platformio run --target upload --upload-port 192.168.1.xxx
```

### Remote ESP32 (replace with actual IP):
```powershell
cd firmware\esp32\boards\remote
platformio run --target upload --upload-port 192.168.1.xxx
```

## Step 6: Wireless Debugging

Monitor serial output over network:
```powershell
# Install netcat (if needed)
# Then connect to port 3232 on ESP32 IP
nc 192.168.1.xxx 3232
```

**Alternative:** Use PlatformIO remote debugging:
```powershell
platformio device monitor --environment esp32dev
```

## Troubleshooting

**WiFi won't connect:**
- Check SSID/password spelling
- Ensure 2.4GHz WiFi (ESP32 doesn't support 5GHz)
- Check router allows new devices

**OTA upload fails:**
- Verify ESP32 is on same network
- Check firewall isn't blocking port 3232
- Try pinging ESP32 IP first

**UART not working:**
- Double-check GPIO16↔GPIO14 and GPIO17↔GPIO15
- Verify GND connection
- Enable UART on Pi (see `/boot/firmware/config.txt`)
- Disable serial console: `sudo systemctl disable serial-getty@serial0.service`

## Raspberry Pi UART Setup

On both Pi 5 and Pi Zero 2W:

```bash
# Edit boot config
sudo nano /boot/firmware/config.txt

# Add these lines:
dtparam=uart0=on
enable_uart=1

# Save and reboot
sudo reboot

# After reboot, disable serial console:
sudo systemctl disable serial-getty@serial0.service
sudo systemctl stop serial-getty@serial0.service
```

Verify UART device exists:
```bash
ls -l /dev/serial0
# Should show: /dev/serial0 -> ttyAMA0
```

## Testing Communication

Once connected, you can test from the Raspberry Pi:

```python
import serial
import time

# Open UART
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

# Send test message (heartbeat 0x20)
test_msg = bytes([0xAA, 0x20, 0x00, 0x20, 0x55])  # Empty heartbeat
ser.write(test_msg)

# Check ESP32 serial output for "Forwarding type=0x20"
```
