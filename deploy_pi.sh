#!/bin/bash
# deploy_pi.sh - Deployment script for Raspberry Pi
# Usage: ./deploy_pi.sh

set -e

REPO_DIR="/home/pi/Raspberry_Pi_01"
BRANCH="master"

# Print current date and time
DATE=$(date)
echo "Deployment started at $DATE"

cd "$REPO_DIR"

echo "Pulling latest code from $BRANCH..."
git fetch origin
git checkout $BRANCH
git pull origin $BRANCH

# Print latest commit info
echo "Current commit: $(git rev-parse HEAD)"
git log -1 --oneline

# Show version info from main scripts
if grep -q APP_VERSION raspberry_pi/robot/robot_controller.py; then
    echo -n "robot_controller.py version: "
    python3 -c "from raspberry_pi.robot.robot_controller import APP_VERSION; print(APP_VERSION)"
fi
if grep -q APP_VERSION raspberry_pi/remote/remote_control.py; then
    echo -n "remote_control.py version: "
    python3 -c "from raspberry_pi.remote.remote_control import APP_VERSION; print(APP_VERSION)"
fi
if grep -q APP_VERSION raspberry_pi/remote/uart_test.py; then
    echo -n "uart_test.py version: "
    python3 -c "from raspberry_pi.remote.uart_test import APP_VERSION; print(APP_VERSION)"
fi

echo "Deployment complete."
