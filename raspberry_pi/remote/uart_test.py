import serial
import time

# Update this to match your UART device and baud rate
UART_PORT = '/dev/serial0'  # or COMx on Windows, or /dev/ttyUSB0
BAUD_RATE = 115200

APP_VERSION = "0.001"
APP_ROLE = "uart_test"


def main():
    try:
        ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
        print(f"Listening on {UART_PORT} at {BAUD_RATE} baud...")
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode(errors='replace').strip()
                if data:
                    print(f"UART RX: {data}")
            time.sleep(0.05)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    print(f"App Version: {APP_VERSION} | Role: {APP_ROLE}")
    main()
