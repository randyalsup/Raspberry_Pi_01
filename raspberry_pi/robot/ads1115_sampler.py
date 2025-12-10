# ads1115_sampler.py
# Standalone script to sample and print ADS1115 readings for debugging


import time
import board
import busio
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn

# Initialize I2C bus and ADS1115
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS1115(i2c, address=0x48)

chan0 = AnalogIn(adc, ADS1115.P0)
chan1 = AnalogIn(adc, ADS1115.P1)
chan2 = AnalogIn(adc, ADS1115.P2)
chan3 = AnalogIn(adc, ADS1115.P3)

print("Starting ADS1115 sampling. Press Ctrl+C to stop.")
try:
    while True:
        v1 = chan0.voltage * 4.0
        v2 = chan1.voltage * 4.0
        v3 = chan2.voltage * 4.0
        amps = chan3.voltage
        print(f"1:{v1:.2f} 2:{v2:.2f} 3:{v3:.2f} 4:{amps:.2f}")
        time.sleep(1)
except KeyboardInterrupt:
    print("Sampling stopped.")
