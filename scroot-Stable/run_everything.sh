#!/usr/bin/env bash
# All-in-one helper to install dependencies, build the Catkin workspace,
# launch roscore and the planner, and stream the actuator CSV log.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$ROOT_DIR/catkin_ws"
LOG_DIR="$ROOT_DIR/.run_logs"
LOG_FILE="${1:-/tmp/actuator_plan.csv}"

mkdir -p "$LOG_DIR"

log() {
  echo "[run_everything] $*"
}

cleanup() {
  log "Stopping background processes..."
  if [[ -n "${CSV_TAIL_PID:-}" ]]; then kill "$CSV_TAIL_PID" 2>/dev/null || true; fi
  if [[ -n "${PLANNER_PID:-}" ]]; then kill "$PLANNER_PID" 2>/dev/null || true; fi
  if [[ -n "${ROSCORE_PID:-}" ]]; then kill "$ROSCORE_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

kill_existing_ros() {
  log "Killing existing ROS processes (roscore/roslaunch/planner)..."
  pkill -f planner_node.py 2>/dev/null || true
  pkill -f roslaunch 2>/dev/null || true
  pkill -f roscore 2>/dev/null || true
  pkill -f rosmaster 2>/dev/null || true
  pkill -f rosout 2>/dev/null || true
}

install_ros_if_missing() {
  if command -v roscore >/dev/null 2>&1; then
    log "ROS detected; skipping apt install."
    return
  fi

  log "Installing ROS Noetic and build helpers (apt-get)..."
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-desktop-full python3-rosdep python3-catkin-tools python3-opencv

  if command -v rosdep >/dev/null 2>&1; then
    rosdep init 2>/dev/null || true
    rosdep update || true
  else
    log "WARNING: rosdep still missing after install; dependency resolution may fail."
  fi
}

bootstrap_deps() {
  log "Sourcing ROS environment..."
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash

  if command -v rosdep >/dev/null 2>&1; then
    log "Resolving package dependencies with rosdep..."
    rosdep install --from-paths "$WS_DIR/src" --ignore-src -r -y || true
  else
    log "Skipping rosdep because it is unavailable."
  fi
}

build_workspace() {
  log "Building Catkin workspace at $WS_DIR ..."
  catkin_make -C "$WS_DIR"
  # shellcheck disable=SC1091
  source "$WS_DIR/devel/setup.bash"
}

start_roscore() {
  log "Launching roscore..."
  roscore > "$LOG_DIR/roscore.log" 2>&1 &
  ROSCORE_PID=$!
  sleep 2
}

start_planner() {
  log "Starting planner_node (log -> $LOG_FILE)..."
  mkdir -p "$(dirname "$LOG_FILE")"
  touch "$LOG_FILE"
  rosrun scooter_control planner_node.py _log_path:="$LOG_FILE" \
    > "$LOG_DIR/planner.log" 2>&1 &
  PLANNER_PID=$!
}

open_csv_stream() {
  log "Streaming CSV updates (tail -f)..."
  tail -f "$LOG_FILE" &
  CSV_TAIL_PID=$!

  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$LOG_FILE" >/dev/null 2>&1 || true
  fi
}

main() {
  log "Working directory: $ROOT_DIR"
  kill_existing_ros
  install_ros_if_missing
  bootstrap_deps
  build_workspace
  start_roscore
  start_planner
  open_csv_stream
  log "Planner running. Press Ctrl+C to stop and clean up."
  wait "$PLANNER_PID"
}

main "$@"
