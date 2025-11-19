#!/usr/bin/env bash
set -euo pipefail

patterns=(
  "sensor_interface_node.py"
  "visualize_sensor_hub.py"
  "dual_yolo_node.py"
  "ai_input_bridge.py"
  "sensor_interface_dynamic.launch"
  "yolo.launch"
  "full_system.launch"
  "roscore"
)

for pattern in "${patterns[@]}"; do
  if pkill -f "$pattern" >/dev/null 2>&1; then
    echo "Stopped processes matching $pattern"
  fi
done

echo "All known sensor/AI processes have been terminated."
