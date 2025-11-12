#!/usr/bin/env python3
"""Simple Raspberry Pi Zero W example program.
Blink an LED on a GPIO pin and print CPU temperature.
"""
import time
import math
import argparse

# gpiozero is used on the Pi. On non-Pi machines this may fail.
try:
    from gpiozero import LED
except Exception:
    # Provide a fake LED for non-Pi test/development
    class LED:
        def __init__(self, pin):
            self.pin = pin
        def on(self):
            print(f"LED {self.pin} ON")
        def off(self):
            print(f"LED {self.pin} OFF")


def round_up_1dp(v: float) -> float:
    """Round up to 1 decimal place (e.g., 1.01 -> 1.1)."""
    return math.ceil(v * 10.0) / 10.0


def get_cpu_temp() -> float | None:
    """Read CPU temperature on Pi if available (degrees C)."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return None


def main(blink_pin: int = 17, blink_count: int = 3) -> None:
    led = LED(blink_pin)
    print(f"Blinking LED on GPIO{blink_pin} x{blink_count}")
    for _ in range(blink_count):
        led.on()
        time.sleep(0.5)
        led.off()
        time.sleep(0.5)

    temp = get_cpu_temp()
    if temp is not None:
        print(f"CPU temp: {round_up_1dp(temp)} C")
    else:
        print("CPU temp: not available")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--blink-pin", type=int, default=17, help="GPIO pin for LED (BCM)")
    parser.add_argument("--count", type=int, default=3, help="Number of blinks")
    args = parser.parse_args()
    main(args.blink_pin, args.count)
