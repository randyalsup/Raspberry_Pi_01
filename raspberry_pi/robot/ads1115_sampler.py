# ads1115_sampler.py
# Standalone script to sample and print ADS1115 readings for debugging

import time
try:
    from Adafruit_ADS1x15 import ADS1115
except ImportError:
    print("Adafruit_ADS1x15 library not found. Install with: pip3 install adafruit-ads1x15")
    exit(1)

adc = ADS1115(address=0x48)
GAIN = 1

print("Starting ADS1115 sampling. Press Ctrl+C to stop.")
try:
    while True:
        readings = [adc.read_adc(ch, gain=GAIN) for ch in range(4)]
        # Convert raw readings to voltages (assuming gain=1, 4.096V range, 16-bit ADC)
        voltages = [r * 4.096 / 32767 for r in readings]
        # Format like robot 2004 LCD: '5V: x.xxV 5V: x.xxV 12V: x.xxV Amps: x.xxA'
        # (Assume channels 0,1 = 5V, channel 2 = 12V, channel 3 = Amps sensor)
        print(f"5V: {voltages[0]:.2f}V   5V: {voltages[1]:.2f}V   12V: {voltages[2]:.2f}V   Amps: {voltages[3]:.2f}A")
        time.sleep(1)
except KeyboardInterrupt:
    print("Sampling stopped.")
