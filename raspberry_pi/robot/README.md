# Raspberry Pi 5 Robot Software

## Requirements

```bash
sudo apt update
sudo apt install python3-pip python3-serial python3-rpi.gpio

pip3 install adafruit-circuitpython-ads1x15
```

## Hardware Setup

### UART Configuration

Edit `/boot/firmware/config.txt`:
```
dtparam=uart0=on
enable_uart=1
```

Disable console on serial:
```bash
sudo systemctl disable serial-getty@serial0.service
sudo reboot
```

### Test UART Connection

```bash
# Send test to ESP32
echo -e "\xAA\x01\x00\x00\x55" > /dev/serial0

# Read from ESP32
cat /dev/serial0
```

### I2C Configuration

Enable I2C:
```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

Test ADS1115:
```bash
sudo i2cdetect -y 1
# Should show 0x48
```

## Pin Connections

### UART to ESP32
- GPIO14 (Pin 8) → ESP32 GPIO16 (RX2)
- GPIO15 (Pin 10) → ESP32 GPIO17 (TX2)
- GND → ESP32 GND

### I2C to ADS1115
- GPIO2 (Pin 3, SDA) → ADS1115 SDA
- GPIO3 (Pin 5, SCL) → ADS1115 SCL
- 3.3V (Pin 1) → ADS1115 VDD
- GND (Pin 9) → ADS1115 GND
- ADS1115 ADDR → GND (sets address to 0x48)

### Motor Drivers (BTS7960)
- GPIO12 (Pin 32) → Left RPWM
- GPIO13 (Pin 33) → Left LPWM
- GPIO18 (Pin 12) → Right RPWM
- GPIO19 (Pin 35) → Right LPWM
- GPIO26 (Pin 37) → Both R_EN + L_EN (enable)

## Running

```bash
cd ~/raspberry_pi/robot
python3 robot_controller.py
```

## Auto-start on Boot

Create systemd service:

```bash
sudo nano /etc/systemd/system/robot.service
```

```ini
[Unit]
Description=Robot Controller
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/raspberry_pi/robot
ExecStart=/usr/bin/python3 /home/pi/raspberry_pi/robot/robot_controller.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable robot.service
sudo systemctl start robot.service
sudo systemctl status robot.service
```

## Testing

### Test ADS1115 Only

```python
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c, address=0x48)
chan0 = AnalogIn(ads, ADS.P0)

print(f"Voltage: {chan0.voltage * 4.0:.2f}V")  # With 4:1 divider
```

### Test Motors Only

```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
pwm = GPIO.PWM(13, 1000)
pwm.start(50)  # 50% duty cycle

time.sleep(2)
pwm.stop()
GPIO.cleanup()
```

### Test UART Only

```python
import serial
import time

ser = serial.Serial('/dev/serial0', 115200)
test_packet = bytes([0xAA, 0x20, 0x05, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x55])
ser.write(test_packet)

time.sleep(0.1)
if ser.in_waiting:
    response = ser.read(ser.in_waiting)
    print("Received:", response.hex())
```

## Troubleshooting

### "Permission denied: '/dev/serial0'"
```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

### "I2C device not found"
- Check wiring
- Verify I2C enabled: `ls /dev/i2c*`
- Scan bus: `sudo i2cdetect -y 1`

### Motors don't move
- Check BTS7960 power (12V connected?)
- Verify enable pin HIGH
- Test PWM output with LED first
- Check motor connections (no shorts)

### No UART communication
- Verify ESP32 is powered and running
- Check TX/RX not swapped
- Test loopback (connect Pi TX to Pi RX)
- Check baud rate matches (115200)

## Performance

- **Loop rate:** 100Hz
- **Telemetry:** 10Hz (every 100ms)
- **Heartbeat:** 1Hz
- **ADS1115 sample rate:** ~860 SPS max
- **UART latency:** ~1ms typical

## Safety Features

### Low Voltage Cutoff
```python
if voltage < 10.5:
    motors.enable(False)
    # Send warning to remote
```

### Watchdog
```python
if time.time() - last_command > 2.0:
    motors.brake()  # No commands for 2 seconds
```

### Emergency Stop
- Hardware button on GPIO4
- Software command via UART
- Immediately disables motor drivers

## Next Steps

- [ ] Test basic UART communication
- [ ] Test ADS1115 voltage reading
- [ ] Test motor control (low speed first!)
- [ ] Integrate all components
- [ ] Add ultrasonic sensors
- [ ] Add UWB ranging (when modules arrive)
- [ ] Implement follow-me mode
