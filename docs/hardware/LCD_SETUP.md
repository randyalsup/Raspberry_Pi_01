# 2004A LCD Display Setup

## Hardware: 2004A-V1.1 (20x4 I2C Character LCD)

### Pin Connections

Both the Remote (Pi Zero 2W) and Robot (Pi 5) use the same connections:

| LCD Pin | Pi Pin | GPIO | Description |
|---------|--------|------|-------------|
| VCC | Pin 2 | 5V | Power |
| GND | Pin 6 | GND | Ground |
| SDA | Pin 3 | GPIO 2 | I2C Data |
| SCL | Pin 5 | GPIO 3 | I2C Clock |

### I2C Addresses

The 2004A typically comes configured at one of these addresses:
- **0x27** (most common)
- **0x3F** (alternate)

The software auto-detects which address is in use.

## Software Installation

### Pi Zero 2W (Remote)

```bash
# Install I2C tools
sudo apt update
sudo apt install -y i2c-tools python3-smbus

# Install Python LCD library
sudo pip3 install RPLCD

# Verify LCD detected
sudo i2cdetect -y 1
# Should show device at 0x27 or 0x3F
```

### Pi 5 (Robot)

```bash
# Install I2C tools
sudo apt update
sudo apt install -y i2c-tools python3-smbus

# Install Python LCD library
sudo pip3 install RPLCD

# Verify LCD detected
sudo i2cdetect -y 1
# Should show both ADS1115 (0x48) and LCD (0x27 or 0x3F)
```

## Display Layout

### Remote (Pi Zero 2W)

```
┌────────────────────┐
│Remote Control v1.0 │ Line 1: Title
│Touch: X:+050 Y:-030│ Line 2: Joystick position
│Motor: L:+450 R:+450│ Line 3: Calculated motor speeds
│TX: CMD_MOTOR 20Hz  │ Line 4: Last transmitted message
└────────────────────┘
```

### Robot (Pi 5)

```
┌────────────────────┐
│Robot v1.0 ACTIVE   │ Line 1: Status (ACTIVE/IDLE)
│Bat: 12.4V  2.3A    │ Line 2: Battery voltage & current
│Mot: L:+450 R:+450  │ Line 3: Actual motor speeds
│RX: CMD_MOTOR 15s   │ Line 4: Last received message
└────────────────────┘
```

## Usage

### Remote Control UI

The LCD updates automatically when motor commands are sent (20Hz rate).

Shows:
- **Line 1**: Application version
- **Line 2**: Normalized joystick position (-100 to +100)
- **Line 3**: Motor speeds being transmitted (-1000 to +1000)
- **Line 4**: Message type and update rate

### Robot Controller

The LCD updates at 5Hz with telemetry data.

Shows:
- **Line 1**: Robot status (ACTIVE when receiving commands)
- **Line 2**: Battery voltage and current from ADS1115
- **Line 3**: Current motor speeds
- **Line 4**: Last received command with timestamp

## Troubleshooting

### LCD Not Detected

```bash
# Check I2C is enabled
sudo raspi-config
# Interface Options → I2C → Enable

# Scan for devices
sudo i2cdetect -y 1

# If nothing shows:
# 1. Check wiring (especially power)
# 2. Try different LCD module (may be faulty)
# 3. Check backpack soldering
```

### Wrong I2C Address

If your LCD uses a different address (e.g., 0x20 or 0x38):

**Remote:** Edit `remote_control_ui.py`, line ~275:
```python
for addr in [0x27, 0x3F, 0x20, 0x38]:  # Add your address
```

**Robot:** Edit `robot_controller.py`, line ~255:
```python
for addr in [0x27, 0x3F, 0x20, 0x38]:  # Add your address
```

### Garbled Display

```python
# Try adjusting the LCD initialization
self.lcd = CharLCD('PCF8574', addr, cols=20, rows=4, 
                   charmap='A00')  # Try 'A00' or 'A02'
```

### Display Too Bright/Dim

Adjust the potentiometer on the I2C backpack (small blue trimmer).

## Benefits

1. **Instant Feedback**: See system status without SSH
2. **Wireless Range Testing**: Monitor signal as you move apart
3. **Protocol Debugging**: Confirm messages are being sent/received
4. **Field Operation**: Both devices show status standalone
5. **Development Aid**: See values updating in real-time

## Performance Impact

- **I2C Speed**: Standard 100kHz (negligible bandwidth)
- **Update Rate**: 5Hz (robot), 20Hz (remote when commanded)
- **CPU Impact**: <1% (write operations are asynchronous)
- **Shared Bus**: No conflicts with ADS1115 (different address)

## Future Enhancements

- Add battery percentage bar graph (custom characters)
- Signal strength indicator (RSSI bars)
- Error/warning icons
- UWB distance display (when modules connected)
- Follow-me mode status indicator
