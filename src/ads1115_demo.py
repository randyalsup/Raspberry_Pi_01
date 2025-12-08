#!/usr/bin/env python3
"""ADS1115 demo for Raspberry Pi using Adafruit CircuitPython libraries.

Requirements on the Pi:
  sudo apt update
  sudo apt install -y python3-pip python3-smbus i2c-tools
  sudo pip3 install adafruit-blinka adafruit-circuitpython-ads1x15

Run on the Pi after wiring the ADS1115 to I2C (SDA, SCL, power, GND):
  python3 ~/ads1115_demo.py

This script reads channel 0 and prints raw and voltage values every second.
"""

import time
import sys

try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
except Exception as e:
    print("Missing required libraries or running on non-Pi platform:", e)
    print("Ensure you installed adafruit-blinka and adafruit-circuitpython-ads1x15 on the Pi.")
    sys.exit(1)


def main():
    print("Starting ADS1115 demo — creating I2C bus")
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    chan0 = AnalogIn(ads, ADS.P0)

    print("ADS1115 address: 0x%02X" % ads.address)
    print("Reading channel 0 (single-ended) — Ctrl-C to stop")
    try:
        while True:
            now = time.strftime('%Y-%m-%d %H:%M:%S')
            print(f"{now} raw={chan0.value} voltage={chan0.voltage:.6f} V")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('\nExiting')


if __name__ == '__main__':
    main()
