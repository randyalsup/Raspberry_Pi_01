# Raspberry Pi ↔ ESP32 UART Protocol Specification

## Overview

This protocol defines communication between:
- **Robot Side:** Raspberry Pi 5 ↔ ESP32 (RO)
- **Remote Side:** Raspberry Pi Zero 2W ↔ ESP32 (RC)

The ESP32 acts as a transparent wireless bridge, forwarding packets between the two Raspberry Pis via ESP-NOW.

## Physical Layer

- **Interface:** UART (Serial)
- **Baud Rate:** 115200 bps
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1
- **Flow Control:** None
- **Pins:**
  - Pi GPIO14 (TXD0) → ESP32 GPIO16 (RX2)
  - Pi GPIO15 (RXD0) → ESP32 GPIO17 (TX2)
  - Common GND

## Message Format

All messages use a simple framed packet structure with CRC for reliability.

```
┌──────┬──────┬────────┬─────────┬──────┬─────┐
│ SYNC │ TYPE │ LENGTH │ PAYLOAD │ CRC8 │ END │
├──────┼──────┼────────┼─────────┼──────┼─────┤
│ 0xAA │ 1B   │ 1B     │ 0-250B  │ 1B   │ 0x55│
└──────┴──────┴────────┴─────────┴──────┴─────┘
```

### Field Definitions

| Field | Size | Description |
|-------|------|-------------|
| **SYNC** | 1 byte | Start marker: `0xAA` |
| **TYPE** | 1 byte | Message type code (see below) |
| **LENGTH** | 1 byte | Payload length (0-250 bytes) |
| **PAYLOAD** | 0-250 bytes | Message-specific data |
| **CRC8** | 1 byte | CRC-8 checksum of TYPE + LENGTH + PAYLOAD |
| **END** | 1 byte | End marker: `0x55` |

**Total overhead:** 5 bytes per message

## Message Types

### From Pi to ESP32 (Commands)

| Type | Code | Direction | Description |
|------|------|-----------|-------------|
| `CMD_MOTOR` | 0x01 | Pi 5 → Robot | Motor control command |
| `CMD_CONFIG` | 0x02 | Both | Configuration update |
| `CMD_QUERY` | 0x03 | Both | Request status/info |
| `CMD_UI` | 0x04 | Pi Zero → Remote | UI event (touch/button) |
| `CMD_MODE` | 0x05 | Both | Operating mode change |

### From ESP32 to Pi (Status/Data)

| Type | Code | Direction | Description |
|------|------|-----------|-------------|
| `STA_TELEMETRY` | 0x10 | Robot → Pi 5 | Sensor readings from remote robot |
| `STA_WIRELESS` | 0x11 | Both | ESP-NOW status (RSSI, pairing) |
| `STA_UWB` | 0x12 | Both | UWB distance/angle data |
| `STA_ERROR` | 0x13 | Both | Error/warning messages |
| `STA_ACK` | 0x14 | Both | Acknowledge receipt |

### Bidirectional

| Type | Code | Direction | Description |
|------|------|-----------|-------------|
| `MSG_HEARTBEAT` | 0x20 | Both | Keepalive (1Hz) |
| `MSG_DEBUG` | 0x21 | Both | Debug/logging |

## Message Payload Formats

### 0x01 CMD_MOTOR (Pi 5 → Robot ESP32)

Control robot motors. This is forwarded via ESP-NOW to remote robot.

```c
struct {
  int16_t left_speed;   // -1000 to +1000 (forward/reverse)
  int16_t right_speed;  // -1000 to +1000
  uint8_t brake;        // 0=coast, 1=active brake
  uint8_t enable;       // 0=disable, 1=enable
} __attribute__((packed));
```
**Length:** 6 bytes

### 0x04 CMD_UI (Pi Zero 2W → Remote ESP32)

User input from touch display or buttons.

```c
struct {
  uint8_t event_type;   // 0=touch, 1=button, 2=gesture
  uint8_t event_id;     // Button ID or gesture type
  int16_t x;            // Touch X coordinate (or 0)
  int16_t y;            // Touch Y coordinate (or 0)
  uint8_t pressure;     // Touch pressure 0-255 (or 0)
} __attribute__((packed));
```
**Length:** 7 bytes

### 0x10 STA_TELEMETRY (Robot ESP32 → Pi 5)

Robot telemetry received via ESP-NOW from remote robot.

```c
struct {
  float battery_voltage;   // Volts
  float battery_current;   // Amps
  float speed_fps;         // Feet per second
  uint8_t status_code;     // 0=OK, 1=BATT_LOW, 2=TERRAIN, etc.
  int8_t rssi;             // ESP-NOW signal strength (dBm)
  uint32_t timestamp;      // Milliseconds since boot
} __attribute__((packed));
```
**Length:** 18 bytes

### 0x11 STA_WIRELESS (ESP32 → Pi)

ESP-NOW wireless status.

```c
struct {
  uint8_t paired;          // 0=not paired, 1=paired
  uint8_t peer_mac[6];     // Peer MAC address
  int8_t rssi;             // Signal strength (dBm)
  uint8_t packet_loss;     // % packet loss (0-100)
  uint32_t last_rx_time;   // ms since last receive
} __attribute__((packed));
```
**Length:** 13 bytes

### 0x12 STA_UWB (ESP32 → Pi)

UWB ranging data (when modules are connected).

```c
struct {
  float distance_left;     // Distance from left UWB module (meters)
  float distance_right;    // Distance from right UWB module (meters)
  float angle_degrees;     // Calculated bearing angle (-180 to +180)
  uint8_t quality;         // Signal quality 0-100
  uint32_t timestamp;      // Milliseconds
} __attribute__((packed));
```
**Length:** 17 bytes

**Note:** Robot has 2 UWB modules (left/right), remote has 1. Robot ESP32 calculates angle and sends to Pi 5.

### 0x20 MSG_HEARTBEAT (Bidirectional)

Keepalive message to detect disconnection.

```c
struct {
  uint32_t uptime_ms;      // Milliseconds since boot
  uint8_t health;          // 0=healthy, 1=warning, 2=error
} __attribute__((packed));
```
**Length:** 5 bytes

**Frequency:** Send every 1 second. If no heartbeat received for 3 seconds, assume disconnection.

## CRC-8 Calculation

**Polynomial:** 0x07 (x^8 + x^2 + x + 1)
**Initial value:** 0x00
**Fields included:** TYPE + LENGTH + PAYLOAD

```c
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}
```

## Example Message (Hex Dump)

**CMD_MOTOR: left=500, right=500, brake=0, enable=1**

```
AA 01 06 F4 01 F4 01 00 01 [CRC] 55
│  │  │  └─────payload────┘  │    │
│  │  │                      │    └─ END (0x55)
│  │  │                      └────── CRC8
│  │  └─────────────────────────── LENGTH (6 bytes)
│  └────────────────────────────── TYPE (0x01)
└───────────────────────────────── SYNC (0xAA)
```

## Error Handling

### Framing Errors
- **Missing SYNC byte:** Discard bytes until `0xAA` found
- **Missing END byte:** Discard packet, wait for next SYNC
- **Length mismatch:** Discard packet

### CRC Errors
- **CRC mismatch:** Discard packet, send `STA_ERROR` back to Pi
- **Retry:** Pi can resend command if no ACK received within 100ms

### Timeout
- **Command timeout:** 500ms - if no response, assume ESP32 busy or disconnected
- **Heartbeat timeout:** 3 seconds - assume UART disconnection

## Communication Flow Examples

### Example 1: Motor Command

```
Pi 5 → ESP32 (RO):   CMD_MOTOR (left=500, right=500)
ESP32 (RO) → Pi 5:   STA_ACK
ESP32 (RO) → ESP32 (RC): [ESP-NOW packet]
ESP32 (RC) → Pi Zero:    STA_TELEMETRY (status from robot)
```

### Example 2: Touch Input

```
Pi Zero → ESP32 (RC):    CMD_UI (touch at x=120, y=200)
ESP32 (RC) → Pi Zero:    STA_ACK
ESP32 (RC) → ESP32 (RO): [ESP-NOW packet]
ESP32 (RO) → Pi 5:       CMD_UI (forwarded)
```

### Example 3: UWB Data

```
ESP32 (RO):              [Reads 2x UWB modules via SPI]
ESP32 (RO):              [Calculates angle from distance delta]
ESP32 (RO) → Pi 5:       STA_UWB (dist_L=5.2m, dist_R=4.8m, angle=52°)
Pi 5:                    [Uses data for follow-me navigation]
```

## Implementation Notes

### Python (Raspberry Pi)

```python
import serial
import struct

class ESP32Protocol:
    SYNC = 0xAA
    END = 0x55
    
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
    
    def send_motor_cmd(self, left, right, brake=0, enable=1):
        payload = struct.pack('<hhBB', left, right, brake, enable)
        self._send_message(0x01, payload)
    
    def _send_message(self, msg_type, payload):
        length = len(payload)
        crc = self._calc_crc([msg_type, length] + list(payload))
        frame = bytes([self.SYNC, msg_type, length]) + payload + bytes([crc, self.END])
        self.ser.write(frame)
    
    def _calc_crc(self, data):
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
        return crc & 0xFF
```

### C++ (ESP32)

```cpp
#define SYNC_BYTE 0xAA
#define END_BYTE 0x55

void processUART() {
  static uint8_t state = 0;  // 0=wait sync, 1=read type, etc.
  static uint8_t msgType, msgLen;
  static uint8_t payload[250];
  static uint8_t idx = 0;
  
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    
    switch(state) {
      case 0:  // Wait for SYNC
        if (b == SYNC_BYTE) state = 1;
        break;
      case 1:  // Read TYPE
        msgType = b;
        state = 2;
        break;
      case 2:  // Read LENGTH
        msgLen = b;
        idx = 0;
        state = (msgLen > 0) ? 3 : 4;
        break;
      case 3:  // Read PAYLOAD
        payload[idx++] = b;
        if (idx >= msgLen) state = 4;
        break;
      case 4:  // Read CRC
        if (verifyCRC(msgType, msgLen, payload, b)) {
          state = 5;
        } else {
          state = 0;  // CRC failed, restart
        }
        break;
      case 5:  // Read END
        if (b == END_BYTE) {
          handleMessage(msgType, payload, msgLen);
        }
        state = 0;
        break;
    }
  }
}
```

## Performance

- **Throughput:** ~11,520 bytes/sec (115200 baud / 10 bits per byte)
- **Latency:** ~1ms per message (typical 10-20 byte packets)
- **Overhead:** 5 bytes per message (4.3% for 115-byte max packet)
- **Recommended rate:** 
  - Commands: as needed (typ. 10-50 Hz)
  - Telemetry: 10 Hz
  - Heartbeat: 1 Hz
  - UWB data: 10 Hz

## Future Extensions

### Optional: Add Sequence Numbers
Add 1-byte sequence number after TYPE field to detect dropped packets.

### Optional: Add Multi-Packet Support
For large data (logs, config files), split into multiple frames with fragment IDs.

### Optional: Add Encryption
Use AES-128 encryption of payload for security (requires shared key).

---

**Version:** 1.0  
**Last Updated:** 2025-12-02
