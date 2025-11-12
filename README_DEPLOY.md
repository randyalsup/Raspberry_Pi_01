Deployment notes:

1) Ensure the Pi has Python 3 and pip installed:
   sudo apt update && sudo apt install -y python3 python3-pip rsync

2) Setup SSH keys from your dev machine to the Pi to avoid password prompts.

3) Run deploy with env variables:
   PI_USER=pi PI_HOST=192.168.1.100 PI_PATH=/home/pi/Raspberry_Pi_01 ./deploy.sh

4) On Pi, run:
   python3 -m pip install --user -r requirements.txt
   python3 src/main.py
