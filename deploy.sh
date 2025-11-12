#!/usr/bin/env bash
# Deploy the Raspberry_Pi_01 folder to the PI using rsync and run setup
# Usage (example):
# PI_USER=pi PI_HOST=192.168.1.100 PI_PATH=/home/pi/Raspberry_Pi_01 ./deploy.sh

set -e
: "${PI_USER:?Need PI_USER}"
: "${PI_HOST:?Need PI_HOST}"
: "${PI_PATH:=/home/${PI_USER}/Raspberry_Pi_01}"

RSYNC_OPTS="-avz --exclude .git --exclude __pycache__"

echo "Deploying to ${PI_USER}@${PI_HOST}:${PI_PATH}"
rsync ${RSYNC_OPTS} ./Raspberry_Pi_01/ "${PI_USER}@${PI_HOST}:${PI_PATH}"

ssh "${PI_USER}@${PI_HOST}" "cd ${PI_PATH} && python3 -m pip install --user -r requirements.txt"

echo "Deployment finished. You can ssh to the Pi and run: python3 src/main.py"
