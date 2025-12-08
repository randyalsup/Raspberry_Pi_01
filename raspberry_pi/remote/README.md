# Robot Remote Control UI

Touch-based interface for Pi Zero 2W with Waveshare 3.5" LCD.

## Features

- Virtual joystick for tank-steering control
- Real-time telemetry display (battery, signal strength)
- 20Hz motor command updates
- Touch-optimized 320x480 interface

## Installation

On Pi Zero 2W:

```bash
# Install dependencies
sudo apt update
sudo apt install python3-pygame python3-serial

# Or use pip
pip3 install -r requirements.txt

# Make executable
chmod +x remote_control_ui.py
```

## Usage

```bash
# Run the UI
python3 remote_control_ui.py

# Or if made executable
./remote_control_ui.py
```

**Controls:**
- Touch and drag the joystick to control robot
- Forward/backward: vertical movement
- Left/right turning: horizontal movement
- Release to stop

**Exit:**
- Press ESC or Q key
- Motors will automatically stop on exit

## Architecture

```
Touch Input → Joystick → Differential Steering → UART → ESP32 → ESP-NOW → Robot
                                                                              ↓
Telemetry Display ← UART ← ESP32 ← ESP-NOW ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ←┘
```

## Configuration

Edit `remote_control_ui.py` to adjust:
- Motor command rate (default: 20Hz)
- Joystick deadzone (default: 10%)
- Display colors and layout
