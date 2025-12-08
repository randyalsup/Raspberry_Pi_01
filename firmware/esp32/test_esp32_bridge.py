#!/usr/bin/env python3
"""
ESP32 Bridge Test Tool
Sends test UART packets to verify ESP32 <-> ESP-NOW <-> ESP32 forwarding
"""

import serial
import time
import struct

SYNC_BYTE = 0xAA
END_BYTE = 0x55

def crc8(data):
    """Calculate CRC-8 checksum"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
    return crc & 0xFF

def send_test_packet(ser, msg_type, payload):
    """Send a framed test packet"""
    length = len(payload)
    crc_data = bytes([msg_type, length]) + payload
    crc = crc8(crc_data)
    
    frame = bytes([SYNC_BYTE, msg_type, length]) + payload + bytes([crc, END_BYTE])
    
    print(f"Sending: Type=0x{msg_type:02X}, Len={length}, Payload={payload.hex()}")
    ser.write(frame)
    return frame

def read_response(ser, timeout=1.0):
    """Try to read a response packet"""
    start = time.time()
    buffer = b''
    
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            buffer += ser.read(ser.in_waiting)
            
            # Look for complete frame
            if len(buffer) >= 5:  # Minimum frame size
                sync_idx = buffer.find(bytes([SYNC_BYTE]))
                if sync_idx >= 0:
                    if sync_idx > 0:
                        buffer = buffer[sync_idx:]
                    
                    if len(buffer) >= 3:
                        msg_type = buffer[1]
                        length = buffer[2]
                        frame_len = 5 + length  # SYNC + TYPE + LEN + PAYLOAD + CRC + END
                        
                        if len(buffer) >= frame_len:
                            frame = buffer[:frame_len]
                            buffer = buffer[frame_len:]
                            
                            # Verify frame
                            if frame[-1] == END_BYTE:
                                payload = frame[3:3+length]
                                crc_data = bytes([msg_type, length]) + payload
                                calc_crc = crc8(crc_data)
                                recv_crc = frame[3+length]
                                
                                if calc_crc == recv_crc:
                                    print(f"Received: Type=0x{msg_type:02X}, Len={length}, Payload={payload.hex()}")
                                    return frame
                                else:
                                    print(f"CRC mismatch: calc={calc_crc:02X}, recv={recv_crc:02X}")
        time.sleep(0.01)
    
    return None

def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python test_esp32_bridge.py <COM_PORT>")
        print("Example: python test_esp32_bridge.py COM3")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print(f"\n=== ESP32 Bridge Test ===")
    print(f"Port: {port}")
    print(f"Connecting...\n")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(2)  # Wait for ESP32 to boot
        
        # Clear buffer
        if ser.in_waiting:
            ser.read(ser.in_waiting)
        
        print("Test 1: Heartbeat message (0x20)")
        print("-" * 50)
        uptime = int(time.time() * 1000) & 0xFFFFFFFF
        health = 0
        payload = struct.pack('<IB', uptime, health)
        send_test_packet(ser, 0x20, payload)
        response = read_response(ser, timeout=2.0)
        if response:
            print("✓ Got response from remote ESP32!\n")
        else:
            print("✗ No response (check if both ESP32s are powered)\n")
        
        time.sleep(1)
        
        print("Test 2: Motor command (0x01)")
        print("-" * 50)
        left_speed = 500
        right_speed = 500
        brake = 0
        enable = 1
        payload = struct.pack('<hhBB', left_speed, right_speed, brake, enable)
        send_test_packet(ser, 0x01, payload)
        response = read_response(ser, timeout=2.0)
        if response:
            print("✓ Motor command forwarded!\n")
        else:
            print("✗ No response\n")
        
        time.sleep(1)
        
        print("Test 3: Simple test message")
        print("-" * 50)
        payload = b'HELLO'
        send_test_packet(ser, 0x21, payload)  # Debug message
        response = read_response(ser, timeout=2.0)
        if response:
            print("✓ Test message forwarded!\n")
        else:
            print("✗ No response\n")
        
        print("\nTest complete!")
        print("\nIf you saw responses, the bridge is working correctly.")
        print("Both ESP32s are forwarding packets via ESP-NOW.")
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("\nTroubleshooting:")
        print("- Check COM port is correct")
        print("- Ensure ESP32 is connected")
        print("- Try unplugging and replugging USB")
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == '__main__':
    main()
