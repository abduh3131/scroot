# Development Report

## Overview
The scooter autonomy stack now runs on Python 3.8 and newer, spans Jetson Orin to desktop GPUs, and offers a friendlier desktop app. Sensor ingestion is now decoupled from hardware by mirroring the ROS `/sensor_hub/data` stream into `~/scroot/runtime_inputs/` so every AI module reads the same fused camera and LiDAR snapshots regardless of how the data arrives. The GUI scrolls smoothly on every tab, applies Jetson-specific CUDA defaults when that hardware is detected, and stores state between sessions. A brand-new **Media Test** tab lets you upload any road clip or still photo, run it through the entire pilot, and save a video with detection boxes, lane cues, and narration painted on top &mdash; now to any folder you pick, and with a Stop button so you can switch clips instantly. Under the hood the openpilot-style lane detector now performs CLAHE/Canny/HSV/LAB preprocessing, caches its bird’s-eye warp, smooths a bounded auto horizon trim, exposes a GUI editor for the four normalized warp points, and draws a translucent lane corridor plus a bird’s-eye inset **purely as an overlay**. The AI handles steering/throttle/braking independently so losing the overlay never changes the control decision. The YOLO stack can be forced to run on CPU when CUDA kernels are missing, a Quadro P520 compatibility option locks YOLO to `cuda:0` on legacy laptops, the advisor narration is a simple checkbox away, and hazards remain capped so the pilot eases back on speed instead of panic-braking every time a video clip briefly misaligns.

## Program Features to Date
1. **Autonomy Pilot Core** – Camera ingestion, YOLO-based object detection, the new perspective-based lane detector, navigator, controller, safety mindset, telemetry logging, and riding companion narration all run inside `AutonomyPilot`.
2. **Safety Arbiter + Command Interface** – The deterministic arbiter in `autonomy/control/arbitration.py` issues ALLOW/AMEND/BLOCK verdicts, while the natural-language parser turns phrases like “drive 2 m forward” into structured goals.
3. **Jetson-Oriented Optimizations** – The GUI auto-detects Jetson Orin hardware, switches to CUDA-ready dependency stacks, lowers resolution/FPS, and defaults to the precision lane profile so inference stays smooth.
4. **Scrollable Setup & Launch Tabs** – Every panel in the GUI shares the same mouse-wheel handler, so you can breeze through large forms on small displays or touchpads.
5. **Lane Visualization Overlay** – The lane detector still rebuilds a bird’s-eye corridor but now uses it strictly for overlays and telemetry, keeping actuator commands tied to object detections and operator goals even when the overlay disappears.
6. **Media Test Harness** – Upload a `.mp4/.mov/.avi/.mkv` clip or `.png/.jpg/.bmp` photo, then click **Generate Overlay** to replay that media through the pilot and save a narrated overlay video anywhere you want (or keep the `logs/media_tests/` default). Hit **Stop** at any time to cancel the export and discard the partial file so you can pick a new clip immediately.
7. **Toggles for Lightweight Runs** – A CPU/auto/CUDA/Quadro-P520 acceleration dropdown keeps YOLO from tripping CUDA errors on unsupported GPUs, and an “Enable Advisor” checkbox lets you silence narration entirely when you want the leanest control loop.
8. **Lane Alignment Editor** – The Launch tab now includes a Lane Alignment & Warp section with an auto horizon trim toggle, manual tilt entry, and four normalized anchor points so you can retune the translucent corridor for any handlebar camera without touching code.
9. **Documentation Refresh** – README, GUI guide, and this report now describe the new toggles, calibration workflow, overlay-only lane detector, and offline replay flow in approachable language.
10. **SensorHub ROS Bridge + Runtime Inputs** – `scroot/bridge/ros_sensor_bridge.py` subscribes to `/sensor_hub/data`, converts the fused camera frame via `CvBridge`, exports LiDAR ranges to `runtime_inputs/lidar.npy`, writes SensorHub metadata (angles, IMU vector, ultrasonic distance, timestamps) to `sensor_meta.json`, throttles logs to once per second, and the pilot consumes those files through the updated camera and LiDAR sensor wrappers.
11. **ROS Workspace Mirror** – `catkin_ws/src/` now contains the operator’s `sensor_interface` package, the `scooter_control` dual YOLO TensorRT node, and the `scroot_bridge` wrapper/launch files so a Jetson Orin can rebuild the exact `/usb_cam` + `/scan` → `/sensor_hub/data` → `/ai_input_bridge` topology shown in the provided `rqt_graph` outputs.

## GUI + Sensor Interface Workflow
Follow these steps whenever you want the desktop GUI to stay in lock-step with the fused `/sensor_hub/data` feed from the `sensor_interface_node`:

1. **Bring the ROS sensor interface online**
   ```bash
   roslaunch sensor_interface sensor_interface_dynamic.launch
   ```
   This node publishes `sensor_interface/msg/SensorHub` messages that contain both the fused camera frame and LiDAR ranges.

2. **Mirror ROS data into runtime inputs**
   ```bash
   ./scroot/bridge/ros_sensor_bridge.py
   ```
   The bridge initializes the `ai_input_bridge` ROS node, subscribes to `/sensor_hub/data`, converts the camera image with `CvBridge`, converts the LiDAR ranges into a NumPy array, and writes the synchronized snapshots to `~/scroot/runtime_inputs/camera.jpg`, `~/scroot/runtime_inputs/lidar.npy`, and `~/scroot/runtime_inputs/sensor_meta.json`. The directory is auto-created, logs are throttled to one line per second, and status updates mention the timestamp of the most recent export so you know data is flowing.

3. **Launch the GUI against the runtime inputs**
   ```bash
   python3 scroot/scooter_app.py
   ```
   The Launch tab’s default camera source already points to `~/scroot/runtime_inputs/camera.jpg`, and the autonomy core now instantiates a `LidarSensor` that reads both `~/scroot/runtime_inputs/lidar.npy` and `sensor_meta.json`. With the bridge running, pressing **Start Pilot** in the GUI immediately consumes the mirrored frames/ranges without touching USB cameras or local LiDAR drivers.

4. **Monitor health from the Launch tab**
   - The Live Feed panel updates as soon as new `.jpg` snapshots arrive. If it freezes, check the bridge log for ROS or file permission issues.
   - The Launch Log includes explicit lines whenever the pilot loads the latest LiDAR snapshot. If you see repeated `FileNotFoundError` entries, make sure the bridge is still running and writing into `~/scroot/runtime_inputs/`.

5. **Shut down cleanly**
   - Click **Stop Pilot** in the GUI before closing the app so telemetry and narration logs flush.
   - Terminate `ros_sensor_bridge.py` with `Ctrl+C`, then stop the ROS launch file. This order prevents dangling writers from holding the runtime files open when you’re done.

With this flow, the GUI, CLI (`python3 scroot/main.py`), and Media Test harness all share the exact same fused sensor snapshots, ensuring consistent perception/planning behavior regardless of whether you are driving live, replaying logs, or developing new features.

## ROS Topology + Architecture Diagram

The mirrored ROS workspace (`catkin_ws/src/`) re-creates the operator’s setup end-to-end:

- `sensor_interface/msg/SensorHub.msg` contains the fused LiDAR ranges plus angle bounds/increments, camera frame, IMU vector, and ultrasonic distance.
- `sensor_interface_node.py` subscribes to `/usb_cam/image_raw` and `/scan` with an `ApproximateTimeSynchronizer`, logs “Topics detected…” when both streams appear, and publishes `/sensor_hub/data` + `/sensor_hub/fused_image` while printing “Published fused /sensor_hub/data …”.
- `dual_yolo_node.py` (package `scooter_control`) loads TensorRT/YOLO weights, subscribes to `/usb_cam/image_raw`, and publishes `/AI/annotated_image` plus `/AI/detections` (custom `DetectionArray` message) strictly for visualization/debugging.
- `scroot_bridge` exposes `ai_input_bridge.py`, a wrapper around `~/scroot/bridge/ros_sensor_bridge.py`, and two launch files (`ai_input_bridge.launch`, `full_system.launch`) so ROS can supervise the bridge alongside the sensor stack.

Expected `rqt_graph` topology (boxes = nodes, circles = topics):

```
  /usb_cam --------------------------> /usb_cam/image_raw ---> sensor_interface_node ---> /sensor_hub/data ---> ai_input_bridge
         \                                                                                 |
          \-> dual_yolo_node -> /AI/annotated_image (viz)                                   V
   rplidarNode --> /scan -----------------------------/                                     runtime_inputs/{camera.jpg,lidar.npy,sensor_meta.json}
                                                                                                     |
                                                                                             AutonomyPilot (python3 scroot/main.py)
```

Dual YOLO stays off the critical path (it never feeds `/sensor_hub/data`), while the bridge is the only subscriber writing runtime files. Launch everything at once via `roslaunch scroot_bridge full_system.launch`, or run the individual commands outlined in the README/GUI guide to match the provided graphs.

## Media Testing Workflow
1. Open the **Media Test** tab after setting up your launch preferences (camera resolution, lane profile, mindset, persona).
2. Use **Browse** to pick any road video or still photo. Images are treated as a short clip so the overlay renderer still produces a video file.
3. Press **Generate Overlay**. The exporter reuses your launch settings, applies Jetson optimizations when needed, and writes the result to either your chosen path or `logs/media_tests/<name>_<timestamp>.mp4` when left blank.
4. Watch the preview panel update in real time. The log explains each step (loading media, running inference, writing frames) so you can diagnose codec or dependency issues instantly.

## Lane Detector: How & Why
The refreshed lane detector lives inside `autonomy/perception/lane_detection.py` and now feeds the visualization/telemetry paths only. Each frame goes through a six-stage pipeline that stays lightweight enough for Python 3.8 laptops and Jetson Orin boards:

1. **Contrast-limited equalization + multi-color masks** keep lane paint visible even in dim or overexposed captures before we mask for white/yellow/green hues.
2. **Gradient, Canny, and adaptive edges** fuse Sobel, Canny, and adaptive-threshold cues so faint tape or grooves still count toward the histogram search.
3. **Auto-trimmed bird’s-eye warp with manual overrides** bounds the horizon shift, smooths it over time, and lets you edit the four normalized points from the GUI to match any camera pitch.
4. **Cached perspective transforms** avoid recomputing homographies when the resolution stays constant, saving CPU/GPU cycles on embedded hardware.
5. **Sliding-window + previous-fit tracker** recreates both rails, falls back to a “search around poly” mode when detections get sparse, and computes curvature, width, and lateral offset in meters.
6. **Projection + visualization** paints the lane corridor back on the live feed, fills it with a translucent band, and produces a mini bird’s-eye inset so operators can sanity-check the detector instantly.

Those metrics mirror the cues openpilot’s `selfdrive/controls/lib/lateral_planner.py` uses, but they stay on the visualization side of the stack. The navigator, controller, and arbiter ignore lane offsets entirely, so AMEND/BLOCK decisions are driven by obstacle detections, commands, and safety caps rather than overlay confidence.

## References
- openpilot `selfdrive/controls/lib/lateral_planner.py` – inspiration for the perception pipeline and lane-overlay visualization.
- Ultralytics YOLOv8 – perception backbone for obstacle detection.

## Testing Notes
- `python -m compileall scroot-Stable/scroot`

## Next Steps
- Extend the media harness with batch processing so large datasets can be processed overnight.
- Add lightweight unit tests around the media exporter to guard against codec regressions on Python 3.8.
- Integrate localization and mapping sources so goal phrases like “drive to the plaza” follow GPS breadcrumbs.
