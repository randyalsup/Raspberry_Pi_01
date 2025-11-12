# Raspberry Pi Zero W Project

This folder contains a simple scaffold for deploying and testing a Python program on a Raspberry Pi Zero W.

Contents:
- `src/main.py` — example Python program that blinks an LED and prints CPU temperature.
- `requirements.txt` — required Python packages for the Pi.
- `deploy.sh` — simple rsync/ssh deploy script (set env variables PI_USER, PI_HOST, PI_PATH).
- `test/test_main.py` — basic unit test for a helper function.

Quick start (on your dev machine):

1. Edit `deploy.sh` or export env vars before running:

```powershell
$env:PI_USER = "pi"
$env:PI_HOST = "192.168.1.100"
$env:PI_PATH = "/home/pi/Raspberry_Pi_01"
./Raspberry_Pi_01/deploy.sh
```

2. On the Pi: install Python3 and pip, then run the service:

```bash
cd ~/Raspberry_Pi_01
python3 -m pip install --user -r requirements.txt
python3 src/main.py --count 5
```

Notes:
- The deploy script uses `rsync` and `ssh`; ensure SSH access and keys are set up.
- Edit `src/main.py` to change the blink pin or behavior.
