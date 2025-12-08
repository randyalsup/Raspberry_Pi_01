#!/usr/bin/env python3
"""Simple demo script to run on the Raspberry Pi.

This prints the hostname and current time, and writes a small file to /tmp
so you can verify it ran.
"""
import socket
from datetime import datetime
import sys


def main():
    host = socket.gethostname()
    now = datetime.utcnow().isoformat() + "Z"
    msg = f"raspi_demo.py ran on {host} at {now} (UTC)\n"
    print(msg, end="")
    try:
        with open('/tmp/raspi_demo_output.txt', 'w', encoding='utf-8') as f:
            f.write(msg)
    except Exception as e:
        print(f"Warning: could not write /tmp/raspi_demo_output.txt: {e}", file=sys.stderr)


if __name__ == '__main__':
    main()
