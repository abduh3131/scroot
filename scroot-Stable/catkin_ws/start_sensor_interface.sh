#!/usr/bin/env bash
# Rebuild the catkin workspace and launch the sensor interface stack.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATKIN_WS="$SCRIPT_DIR"

if [[ ! -d "$CATKIN_WS/src" ]]; then
  echo "This script must live in the catkin_ws directory." >&2
  exit 1
fi

cd "$CATKIN_WS"

if [[ -x ./kill_sensors.sh ]]; then
  ./kill_sensors.sh || true
fi

echo "[1/4] Building catkin workspace..."
catkin_make

echo "[2/4] Sourcing devel/setup.bash"
source devel/setup.bash

echo "[3/4] Launching sensor_interface_dynamic.launch"
roslaunch sensor_interface sensor_interface_dynamic.launch
