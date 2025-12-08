# Hardware Wiring Guide

## Robot Side: Raspberry Pi 5 + ESP32-D (RO)

### Power Distribution
```
12V Battery (3S LiPo)
├─→ BTS7960 Motor Drivers (VCC)
├─→ Buck Converter → 5V → Raspberry Pi 5
└─→ Voltage Divider Network → ADS1115 (battery monitor)
```

### ESP32 RO Connections

#### 1. UART to Raspberry Pi 5
| ESP32 Pin | Pi 5 Pin | Function |
|-----------|----------|----------|
| GPIO16 (RX2) | GPIO14 (TXD0) | Pi → ESP32 |
| GPIO17 (TX2) | GPIO15 (RXD0) | ESP32 → Pi |
| GND | GND | Common ground |

**Pi 5 Setup:**
```bash
# Enable UART in /boot/firmware/config.txt
dtparam=uart0=on
enable_uart=1

# Disable console on serial0
sudo systemctl disable serial-getty@serial0.service
```

#### 2. I2C Bus (ADS1115 + Sensors)
| ESP32 Pin | Device | Notes |
|-----------|--------|-------|
| GPIO21 (SDA) | ADS1115 SDA | Pull-up 4.7kΩ to 3.3V |
| GPIO22 (SCL) | ADS1115 SCL | Pull-up 4.7kΩ to 3.3V |
| GPIO21/22 | VL53L5CX (optional) | Shared I2C bus |
| GPIO21/22 | IMU (optional) | Shared I2C bus |

**ADS1115 I2C Address:** 0x48 (ADDR pin to GND)

#### 3. ADS1115 Analog Inputs

**Channel A0: Battery Voltage Monitor**
```
12V Battery ──┬── 30kΩ resistor ──┬── ADS1115 A0
              │                    │
              │                    ├── 10kΩ resistor ── GND
              │                    │
              └──────────────────────── (3V max at A0 for 12V input)

Calculation: Vbatt = Vmeasured × 4.0
```

**Channel A1: Current Monitor**
```
Current Sensor (e.g., ACS712) → Output voltage (0-5V)
      ↓
   Voltage Divider (4:1)
      ↓
   ADS1115 A1 (0-1.25V = 0-5A equivalent)

Calculation: Current = Vmeasured × 4.0 × (sensor_scale)
```

**Channel A2-A3:** Reserved for 5V/3.3V rail monitoring

#### 4. Motor Control (BTS7960 x2)
| ESP32 Pin | Motor Driver | Function |
|-----------|--------------|----------|
| GPIO25 | Left RPWM | Reverse PWM |
| GPIO26 | Left LPWM | Forward PWM |
| GPIO27 | Right RPWM | Reverse PWM |
| GPIO32 | Right LPWM | Forward PWM |
| GPIO33 | Both R_EN + L_EN | Enable (HIGH to run) |

**BTS7960 Wiring per driver:**
```
VCC → 12V battery
GND → Common ground
B+ → Motor positive
B- → Motor negative
RPWM → ESP32 (reverse PWM)
LPWM → ESP32 (forward PWM)
R_EN → ESP32 GPIO33 (enable)
L_EN → ESP32 GPIO33 (enable)
R_IS, L_IS → Not connected (current sense unused)
```

#### 5. Ultrasonic Sensors (HC-SR04)
| ESP32 Pin | Sensor | Function |
|-----------|--------|----------|
| GPIO18 | Front TRIG | Trigger pulse |
| GPIO19 | Front ECHO | Echo return |
| GPIO23 | Rear TRIG | Trigger pulse |
| GPIO5 | Rear ECHO | Echo return |

**HC-SR04 Wiring:**
- VCC → 5V (from Pi or buck converter)
- GND → Common ground
- TRIG → ESP32 GPIO
- ECHO → ESP32 GPIO (via 1kΩ + 2kΩ voltage divider if sensor outputs 5V)

#### 6. Status & Safety
| ESP32 Pin | Device | Function |
|-----------|--------|----------|
| GPIO2 | LED | Onboard status LED |
| GPIO4 | Button | Emergency stop (INPUT_PULLUP) |

---

## Remote Side: Raspberry Pi Zero 2W + ESP32-D (RC)

### Power Distribution
```
USB Power Bank (5V)
├─→ Raspberry Pi Zero 2W
└─→ ESP32-D (via USB or 3.3V pin)
```

### ESP32 RC Connections

#### 1. UART to Raspberry Pi Zero 2W
| ESP32 Pin | Pi Zero Pin | Function |
|-----------|-------------|----------|
| GPIO16 (RX2) | GPIO14 (TXD0) | Pi → ESP32 |
| GPIO17 (TX2) | GPIO15 (RXD0) | ESP32 → Pi |
| GND | GND | Common ground |

**Pi Zero 2W Setup:** (same as Pi 5)

#### 2. Control Buttons
| ESP32 Pin | Button | Function |
|-----------|--------|----------|
| GPIO18 | Forward | D-pad up |
| GPIO19 | Backward | D-pad down |
| GPIO20 | Left | D-pad left |
| GPIO23 | Right | D-pad right |
| GPIO25 | Action A | Boost/Turbo |
| GPIO26 | Action B | Mode switch |
| GPIO4 | E-Stop | Emergency stop |

**Button wiring:** Each button pulls GPIO to GND when pressed (use INPUT_PULLUP)

#### 3. Feedback Outputs
| ESP32 Pin | Device | Function |
|-----------|--------|----------|
| GPIO2 | LED | Status indicator |
| GPIO32 | Buzzer | Alert sounds |
| GPIO33 | Vibration motor | Haptic feedback (optional) |

#### 4. Optional Analog Controls
| ESP32 Pin | Device | Function |
|-----------|--------|----------|
| GPIO34 | Potentiometer | Throttle slider (0-3.3V) |
| GPIO35 | Potentiometer | Steering trim (0-3.3V) |

---

## Component Specifications

### ADS1115 Configuration
- **Resolution:** 16-bit (32768 levels)
- **Voltage Range:** ±4.096V (configurable)
- **Sample Rate:** 860 SPS (samples per second)
- **I2C Address:** 0x48 (ADDR → GND)

### Voltage Divider Network (4:1 ratio)
**For 12V → 3V:**
- R1 = 30kΩ (high side)
- R2 = 10kΩ (low side)
- Ratio = (R1 + R2) / R2 = 40k / 10k = 4.0

**Calculation formula:**
```cpp
float voltage = ads.readVoltage(0) * 4.0;  // Channel A0
float current = ads.readVoltage(1) * 4.0;  // Channel A1 (scaled by sensor)
```

### BTS7960 Specifications
- **Voltage:** 5.5V to 27V (12V nominal)
- **Current:** 43A max, 10A continuous per driver
- **PWM Frequency:** 1kHz recommended
- **Logic:** 3.3V/5V compatible inputs

### HC-SR04 Specifications
- **Voltage:** 5V
- **Range:** 2cm to 400cm
- **Angle:** 15° cone
- **Echo Output:** 5V (use voltage divider to 3.3V for ESP32 safety)

---

## Safety Notes

1. **Common Ground:** All devices must share common ground
2. **Voltage Levels:** ESP32 is 3.3V logic - use voltage dividers for 5V signals
3. **Power Isolation:** Keep motor power separate from logic power
4. **Fuses:** Add 10A fuse on 12V battery line
5. **E-Stop:** Hardware emergency stop should cut motor driver enable line
6. **Polarity Protection:** Use diodes on motor driver inputs

---

## Testing Checklist

- [ ] Verify 3.3V on ESP32 pins with multimeter
- [ ] Test UART communication (Pi ↔ ESP32) with echo test
- [ ] Verify I2C devices detected (`i2cdetect` on Pi, `Wire.scan()` on ESP32)
- [ ] Test ADS1115 readings with known voltage source
- [ ] Verify voltage divider accuracy (12V → 3V)
- [ ] Test motor drivers with low PWM duty cycle first
- [ ] Verify ultrasonic sensors measure correctly
- [ ] Test emergency stop button functionality
- [ ] Check for ground loops or noise on analog readings
