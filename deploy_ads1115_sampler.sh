#!/bin/bash
# Deploy ads1115_sampler.py to the robot's deployment directory

ROBOT_DEPLOY_DIR="/home/pi/Raspberry_Pi_01/raspberry_pi/robot/"
SRC_FILE="$(dirname "$0")/src/ads1115_sampler.py"

if [ ! -f "$SRC_FILE" ]; then
  echo "Source file not found: $SRC_FILE"
  exit 1
fi

scp "$SRC_FILE" "pi@robot:$ROBOT_DEPLOY_DIR"
echo "ads1115_sampler.py deployed to $ROBOT_DEPLOY_DIR on robot."
