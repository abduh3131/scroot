#!/usr/bin/env bash
# Launch the entire ROS + AI + GUI pipeline inside a tmux session.
set -euo pipefail

SESSION_NAME="scroot_stack"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATKIN_WS="$REPO_ROOT/catkin_ws"
SCROOT_DIR="$REPO_ROOT/scroot"
WINDOW_INDEX=0

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required for run_full_stack.sh" >&2
  exit 1
fi

# Determine the GUI command and optional virtualenv activation.
GUI_CMD="python3 scooter_app.py"
if [[ -n "${SCROOT_VENV:-}" && -f "${SCROOT_VENV}/bin/activate" ]]; then
  GUI_CMD="source ${SCROOT_VENV}/bin/activate && ${GUI_CMD}"
elif [[ -f "$SCROOT_DIR/.venv/bin/activate" ]]; then
  GUI_CMD="source $SCROOT_DIR/.venv/bin/activate && ${GUI_CMD}"
fi

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  tmux kill-session -t "$SESSION_NAME"
fi

start_window() {
  local name="$1"
  local workdir="$2"
  local cmd="$3"
  if [[ $WINDOW_INDEX -eq 0 ]]; then
    tmux new-session -d -s "$SESSION_NAME" -n "$name" -c "$workdir" "bash -lc '$cmd'"
  else
    tmux new-window -t "$SESSION_NAME" -n "$name" -c "$workdir" "bash -lc '$cmd'"
  fi
  WINDOW_INDEX=$((WINDOW_INDEX + 1))
}

# Step A – Build, source, and launch the sensor interface.
start_window "Sensors" "$CATKIN_WS" "./start_sensor_interface.sh"

# Step C – Visualizer.
start_window "Visualizer" "$CATKIN_WS" "source devel/setup.bash && rosrun sensor_interface visualize_sensor_hub.py"

# Step D – Dual YOLO node.
start_window "DualYOLO" "$CATKIN_WS" "source devel/setup.bash && rosrun scooter_control dual_yolo_node.py"

# Bridge SensorHub into AutonomyPilot.
start_window "AI Bridge" "$CATKIN_WS" "source devel/setup.bash && rosrun scroot_bridge ai_input_bridge.py"

# GUI entry point inside the scroot folder.
start_window "GUI" "$SCROOT_DIR" "$GUI_CMD"

tmux select-window -t "$SESSION_NAME:0"
tmux attach-session -t "$SESSION_NAME"
