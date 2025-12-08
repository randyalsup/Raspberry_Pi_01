#!/usr/bin/env python3
"""
Simple UART echo script for testing
Receives data and immediately echoes it back
Run this on Pi Zero 2W (remote)
"""

import serial
import time

port = '/dev/serial0'

print(f"=== UART Echo Test ===")
print(f"Port: {port}")
print(f"Listening and echoing back everything received...")
print(f"Press Ctrl+C to exit\n")

try:
    ser = serial.Serial(port, 115200, timeout=0.1)
    
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"RX ({len(data)} bytes): {data.hex()}")
            ser.write(data)
            print(f"TX ({len(data)} bytes): {data.hex()}\n")
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
except Exception as e:
    print(f"Error: {e}")
