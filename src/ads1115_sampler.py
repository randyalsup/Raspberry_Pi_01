#!/usr/bin/env python3
from __future__ import annotations
"""
ads1115_sampler.py

Simple continuous sampler for ADS1115 (smbus2).

Features:
- Auto-detects first /dev/i2c-* bus or accepts --bus
- Samples specified channels (default AIN0) in single-shot mode
- Handles SMBus endianness and prints CSV lines: timestamp,channel,raw,signed,volts
- Appends to an output CSV file (default stdout)

Usage examples:
  python3 src/ads1115_sampler.py --interval 1.0 --channels 0 1 --output readings.csv
  python3 src/ads1115_sampler.py --bus 1 --channels 0 --interval 0.5

Requires: smbus2 (pip install smbus2)
"""

import argparse
import glob
import os
import sys
import time
from datetime import datetime
from smbus2 import SMBus


def find_i2c_buses() -> list[int]:
    paths = glob.glob("/dev/i2c-*")
    buses = []
    for p in paths:
        try:
            num = int(os.path.basename(p).split("-")[-1])
            buses.append(num)
        except Exception:
            continue
    buses.sort()
    return buses


def swap_word(v: int) -> int:
    return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)


def to_signed_16(v: int) -> int:
    return v if v < 0x8000 else v - 0x10000


# PGA options map name->(bits, volts)
PGA_OPTIONS = {
    6.144: (0b000, 6.144),
    4.096: (0b001, 4.096),
    2.048: (0b010, 2.048),
    1.024: (0b011, 1.024),
    0.512: (0b100, 0.512),
    0.256: (0b101, 0.256),
}


def build_config_single_shot(channel: int, pga_bits: int, dr_bits: int) -> int:
    # ADS1115 config register fields
    OS = 1 << 15
    MUX = (0x4 + channel) & 0x7  # single-ended: 100..111 for AIN0..AIN3
    MUX_bits = (MUX & 0x7) << 12
    PGA_bits = (pga_bits & 0x7) << 9
    MODE_single = 1 << 8
    DR_bits = (dr_bits & 0x7) << 5
    COMP_QUE_disable = 0b11
    cfg = OS | MUX_bits | PGA_bits | MODE_single | DR_bits | COMP_QUE_disable
    return cfg


def sample_loop(bus_num: int, addr: int, channels: list[int], pga_volts: float, dr: int, interval: float, outfp):
    pga_bits, pga_v = PGA_OPTIONS.get(pga_volts, PGA_OPTIONS[4.096])
    # map dr Hz to dr_bits (supported sample rates approximate)
    # use common ADS1115 DR codes: 8,16,32,64,128,250,475,860 -> bits 0..7
    DR_MAP = {8:0,16:1,32:2,64:3,128:4,250:5,475:6,860:7}
    dr_bits = DR_MAP.get(dr, 4)
    conv_time = 1.0 / (dr if dr > 0 else 128)

    with SMBus(bus_num) as bus:
        print(f"Sampling ADS1115 at 0x{addr:02x} on /dev/i2c-{bus_num}: channels={channels}, interval={interval}s, PGA=±{pga_v}V, DR={dr} SPS")
        # write CSV header if file empty
        if outfp is not sys.stdout:
            need_header = (not os.path.exists(outfp.name)) or (os.path.getsize(outfp.name) == 0)
        else:
            need_header = True
        if need_header:
            print("timestamp,channel,raw_hex,raw_signed,volts", file=outfp)
            outfp.flush()

        try:
            while True:
                t = datetime.utcnow().isoformat() + "Z"
                for ch in channels:
                    cfg = build_config_single_shot(ch, pga_bits, dr_bits)
                    try:
                        bus.write_word_data(addr, 0x01, swap_word(cfg))
                    except Exception as e:
                        print(f"write config failed for ch{ch}: {e}", file=sys.stderr)
                        continue
                    time.sleep(max(conv_time * 1.2, 0.01))
                    try:
                        raw = swap_word(bus.read_word_data(addr, 0x00))
                    except Exception as e:
                        print(f"read conversion failed for ch{ch}: {e}", file=sys.stderr)
                        raw = 0
                    s = to_signed_16(raw)
                    volts = s * pga_v / 32768.0
                    print(f"{t},{ch},0x{raw:04x},{s},{volts:.6f}", file=outfp)
                    outfp.flush()
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nSampler stopped by user")


def parse_args():
    p = argparse.ArgumentParser(description="ADS1115 continuous sampler")
    p.add_argument("--bus", type=int, help="I2C bus number (e.g. 1)")
    p.add_argument("--addr", type=lambda x: int(x,0), default=0x48, help="I2C address (default 0x48)")
    p.add_argument("--channels", nargs="*", type=int, default=[0], help="Channels to sample (0-3)")
    p.add_argument("--interval", type=float, default=1.0, help="Interval between sample rounds (seconds)")
    p.add_argument("--pga", type=float, default=4.096, help="PGA full-scale volts (6.144,4.096,2.048,1.024,0.512,0.256)")
    p.add_argument("--dr", type=int, default=128, help="Data rate in SPS (8,16,32,64,128,250,475,860)")
    p.add_argument("--output", type=str, help="Output CSV file (default stdout)")
    return p.parse_args()


def main():
    args = parse_args()
    buses = find_i2c_buses()
    if not buses:
        print("No /dev/i2c-* devices found. Make sure i2c-dev is loaded.")
        return
    bus_num = args.bus if args.bus is not None else buses[0]
    outfp = open(args.output, "a") if args.output else sys.stdout
    try:
        sample_loop(bus_num, args.addr, args.channels, args.pga, args.dr, args.interval, outfp)
    finally:
        if outfp is not sys.stdout:
            outfp.close()


if __name__ == '__main__':
    main()