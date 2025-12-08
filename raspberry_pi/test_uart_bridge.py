#!/usr/bin/env python3
"""
UART Bridge Test Script
Tests communication: Pi → ESP32 → ESP-NOW → ESP32 → Pi

Run this on Raspberry Pi 5 to test the bridge.
"""

import serial
import struct
import time
import sys

def crc8(data):
    """Calculate CRC-8 checksum (polynomial 0x07)"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc = crc << 1
        crc &= 0xFF
    return crc

def send_message(ser, msg_type, payload):
    """Send a framed UART message"""
    length = len(payload)
    
    # Build CRC data: type + length + payload
    crc_data = bytes([msg_type, length]) + payload
    crc = crc8(crc_data)
    
    # Build frame: SYNC + TYPE + LENGTH + PAYLOAD + CRC + END
    frame = bytes([0xAA, msg_type, length]) + payload + bytes([crc, 0x55])
    
    ser.write(frame)
    print(f"Sent: Type=0x{msg_type:02X}, Len={length}, Payload={payload.hex()}, CRC=0x{crc:02X}")
    return frame

def read_response(ser, timeout=2.0):
    """Read and parse a response frame"""
    start_time = time.time()
    state = 0  # 0=wait_sync, 1=type, 2=length, 3=payload, 4=crc, 5=end
    msg_type = 0
    msg_len = 0
    payload = bytearray()
    payload_idx = 0
    
    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            b = ser.read(1)[0]
            
            if state == 0:  # Wait for SYNC
                if b == 0xAA:
                    state = 1
            elif state == 1:  # Read type
                msg_type = b
                state = 2
            elif state == 2:  # Read length
                msg_len = b
                payload = bytearray()
                payload_idx = 0
                state = 3 if msg_len > 0 else 4
            elif state == 3:  # Read payload
                payload.append(b)
                payload_idx += 1
                if payload_idx >= msg_len:
                    state = 4
            elif state == 4:  # Read CRC
                crc_data = bytes([msg_type, msg_len]) + bytes(payload)
                expected_crc = crc8(crc_data)
                if b == expected_crc:
                    state = 5
                else:
                    print(f"CRC mismatch! Got 0x{b:02X}, expected 0x{expected_crc:02X}")
                    state = 0
            elif state == 5:  # Check END byte
                if b == 0x55:
                    print(f"✓ Received: Type=0x{msg_type:02X}, Len={msg_len}, Payload={bytes(payload).hex()}")
                    return (msg_type, bytes(payload))
                state = 0
        else:
            time.sleep(0.01)
    
    return None

def main():
    port = '/dev/serial0'  # Default UART on Raspberry Pi
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"=== ESP32 Bridge Test (Raspberry Pi) ===")
    print(f"Port: {port}")
    print(f"Opening serial port...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(1)  # Let ESP32 settle
        print("Connected!\n")
        
        # Test 1: Heartbeat message
        print("Test 1: Heartbeat (0x20)")
        print("-" * 50)
        uptime = int(time.time() * 1000) & 0xFFFFFFFF
        health = 0xE1  # All healthy
        payload = struct.pack('<IB', uptime, health)
        send_message(ser, 0x20, payload)
        
        response = read_response(ser, timeout=2.0)
        if response:
            print("✓ Bridge working!\n")
        else:
            print("✗ No response\n")
        
        # Test 2: Motor command
        print("Test 2: Motor Command (0x01)")
        print("-" * 50)
        left_speed = 500
        right_speed = 500
        brake = 0
        enable = 1
        payload = struct.pack('<hhBB', left_speed, right_speed, brake, enable)
        send_message(ser, 0x01, payload)
        
        response = read_response(ser, timeout=2.0)
        if response:
            print("✓ Bridge working!\n")
        else:
            print("✗ No response\n")
        
        # Test 3: Debug message
        print("Test 3: Debug Message (0x21)")
        print("-" * 50)
        payload = b"HELLO FROM PI5"
        send_message(ser, 0x21, payload)
        
        response = read_response(ser, timeout=2.0)
        if response:
            print("✓ Bridge working!\n")
        else:
            print("✗ No response\n")
        
        print("=" * 50)
        print("Test complete!")
        print("\nIf you saw responses, the bridge is working correctly.")
        print("Both ESP32s are forwarding packets via ESP-NOW.")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("\nTroubleshooting:")
        print("- Check UART is enabled: /boot/firmware/config.txt")
        print("- Disable serial console: sudo systemctl disable serial-getty@serial0.service")
        print("- Verify wiring: GPIO14(TX)→ESP32 GPIO16(RX), GPIO15(RX)←ESP32 GPIO17(TX)")
        print("- Check /dev/serial0 exists")
        sys.exit(1)

if __name__ == '__main__':
    main()
