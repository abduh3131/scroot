# Planner Integration Report

## Overview
- The existing Catkin graph remains unchanged: camera and LiDAR data are fused by `sensor_interface_node`, detections are produced by `dual_yolo_node`, and the annotated video stream is published on `/AI/annotated_image` alongside `/AI/detections`.
- A new lightweight, rule-based `planner_node` attaches to those AI topics, computes actuator targets (throttle, steering, brake), publishes them to `/planner_node/actuator_plan`, and logs every decision to a CSV file for the downstream MCU bridge.
- No existing topics or message types were modified; the planner is additive and runs whenever the Catkin workspace is active.

## Planner Logic
- Synchronizes the annotated image and YOLO detections via `ApproximateTimeSynchronizer` to keep the perception and planning clocks aligned without blocking the pipeline.
- Picks the nearest detection with a valid distance; if no ranges exist, it locks onto the detection closest to the image center.
- Steering is a clipped proportional response to the horizontal offset of the chosen object center relative to the image center.
- Throttle/brake use distance-based gating: stop inside `~stop_distance`, scale throttle and counter-brake inside `~caution_distance`, and run at `~base_throttle` otherwise.
- Every published `ActuatorCommand` is mirrored to the CSV log with timestamp, throttle, steering, brake, label, and estimated distance.

## How to Run
**One-shot runner (recommended):**
- From the repo root (`scroot-Stable`), execute:
  ```bash
  ./run_everything.sh /tmp/actuator_plan.csv
  ```
  - Kills any existing ROS masters/nodes, installs ROS/rosdep if missing, builds `catkin_ws`, launches `roscore` and the fast `planner_node`, then streams the actuator CSV live (and opens it via `xdg-open` when available).
  - Override the CSV path by passing a different first argument.

**Manual steps:**
1. Build the workspace (from `catkin_ws`):
   ```bash
   catkin_make
   source devel/setup.bash
   ```
2. Launch perception as usual (e.g., camera, LiDAR, YOLO). Ensure `/AI/annotated_image` and `/AI/detections` are active.
3. Start the planner node:
   ```bash
   rosrun scooter_control planner_node.py _log_path:=/tmp/actuator_plan.csv
   ```
   - Outputs actuator commands on `/planner_node/actuator_plan` using the new `ActuatorCommand` message.
   - Writes CSV logs to the configured `_log_path` (default `/tmp/actuator_plan.csv`).

## Configuration
- Key parameters (ROS private params): `~max_steering`, `~base_throttle`, `~caution_distance`, `~stop_distance`, `~log_path`.
- Parameters can be provided via launch files or command-line remapping; defaults favor fast computation with conservative stopping behavior.

## Notes
- The planner avoids heavy image processing—only bounding-box geometry and distances are used—keeping latency low on constrained hardware.
- The CSV logger is opened in append mode so logs survive node restarts without overwriting prior runs.
