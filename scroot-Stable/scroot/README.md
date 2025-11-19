# Autonomous Scooter Pilot

This package contains a self-contained autonomous driving stack tailored for lightweight vehicles such as scooters. The system is designed to run on NVIDIA Jetson devices as well as standard Ubuntu 24.04 laptops and now ingests fused camera + LiDAR feeds from the ROS `sensor_interface_node` via the `/sensor_hub/data` topic. The dedicated **AI Input Bridge** (`bridge/ai_input_bridge.py`) subscribes to that topic, converts each SensorHub message into the existing `SensorSample`/`LidarSnapshot` types, and feeds them directly into `AutonomyPilot` so the AI never opens USB cameras or LiDAR drivers on its own. When you enable mirroring (default), the bridge also writes the snapshots into `~/scroot/runtime_inputs/` for the GUI and media tools. It exposes raw actuator values (`steer`, `throttle`, `brake`) that you can feed directly into your hardware interface. All commands below assume you are inside the `scroot` directory.

## Features

- SensorHub ingestion pipeline that streams `/sensor_hub/data` directly into `AutonomyPilot` so the navigator, arbiter, and controller react to the fused ROS feed with no extra capture loops.
- Companion ROS bridge (`scroot/bridge/ai_input_bridge.py`) that subscribes to `/sensor_hub/data`, converts the image via `CvBridge`, feeds it into the AI, optionally mirrors LiDAR/camera snapshots into `~/scroot/runtime_inputs/`, and throttles logs to one per second.
- Mirrored ROS1 workspace under `catkin_ws/src/` that ships the operator’s `sensor_interface` package, the dual YOLO TensorRT node (`scooter_control`), and the launch-ready `scroot_bridge` wrapper so the Jetson can rebuild the entire topology in one go.
- Real-time object detection powered by Ultralytics YOLO models (default: `yolov8n.pt`).
- Hybrid navigation that fuses obstacle density analysis with natural-language operator goals.
- Layered arbitration stack (Navigator → Arbiter → SafetyGate) that can ALLOW, AMEND, or BLOCK commands every control tick.
- Optional safety mindset profiles that cap maximum speed / clearance and adapt to perception uncertainty or sensor degradation.
- Vehicle envelope modeling with user-defined dimensions and camera calibration to keep commands within real clearance limits.
- Riding companion narration that explains each arbiter decision without blocking the control loop.
- Advanced bird’s-eye lane detector (mirrors openpilot’s lateral planner) that equalizes lighting, uses a fixed downward-tilted perspective warp tuned for scooter cameras, and only renders a translucent overlay when it actually sees a lane.
- Lane visualization is purely cosmetic—the AI handles steering/throttle/brake without any lane-bias injection, so control stays consistent even when the detector loses confidence.
- Configurable lane-warp calibration panel that lets you toggle the automatic horizon trim, manually drag the four normalized warp points, and keep the translucent lane corridor glued to the pavement on any camera height.
- CPU/GPU acceleration toggle plus an advisor switch so you can run entirely on CPU hardware, enable a Quadro P520 compatibility mode, and silence narration when you need the lightest loop.
- Live launch dashboard with camera feed, optional lane-corridor overlays, and arbiter status plus a bird’s-eye mini-map.
- Scroll-friendly desktop GUI with Setup, Launch, and Media Test tabs so you can configure hardware, run the live pilot, or upload recorded drives for offline evaluation.
- Command parser that understands phrases such as "drive to the plaza", "drive 2 m forward", or "turn right" and feeds them into the navigator.
- Structured telemetry exports (`logs/command_state.json`, `logs/telemetry.jsonl`, `logs/incidents.jsonl`) for auditing and supervision.
- Modular design that allows future sensors (ultrasonic, LiDAR, depth) to feed into the navigator without architectural changes.

## Quick Start

### GUI workflow

1. **Initialize the managed environment (first launch only)** so the GUI can install everything inside `.venv/` without tripping PEP 668 protections:

   ```bash
   python setup_scroot.py --skip-models  # installs dependencies into .venv/
   source .venv/bin/activate             # or .venv\Scripts\activate on Windows
   python -m pip install --upgrade pip
   python -m pip install -r autonomy/requirements.txt
   ```

   On subsequent runs you only need to activate the virtualenv if it is not already active.

2. **Run the scooter desktop app** and follow the guided setup:

   ```bash
   python scooter_app.py
   ```

   The application now bootstraps itself: it scans your hardware, detects whether you are running native Ubuntu, Ubuntu-on-WSL, or Jetson JetPack, initializes the configuration file if this is your first run, and installs the dependency profile that best matches your compute tier. If the host Python refuses system-wide installs (PEP 668 “externally managed environment”), the bootstrapper automatically provisions `.venv/` and re-runs the install inside that sandbox so the GUI still opens with a single command. Subsequent launches reuse the cached install unless you switch profiles, so you can jump straight into the GUI. You can override the defaults at any time and re-run the setup if you swap hardware. The **Vehicle Envelope** panel lets you describe your scooter/cart, enter width/length/height, set a preferred side margin, and calibrate how wide the vehicle looks in the camera at a known distance so the safety arbiter understands real clearance.

3. **Open the *Launch* tab** to start the autonomy stack. Choose your camera index, tweak the frame size or FPS if needed, and press **Start Pilot**. Logs from the pilot appear in real time inside the GUI. The live feed shows the current frame, YOLO bounding boxes, an optional lane corridor overlay (only when detections are confident), safety-arbiter verdicts, and now a LiDAR telemetry row that summarizes the closest/median fused ranges plus ultrasonic distance pulled from `~/scroot/runtime_inputs/lidar.npy` + `sensor_meta.json` so you can verify the SensorHub topic is mirrored correctly. Use the messaging panel to monitor ALLOW/AMEND/BLOCK decisions and send natural-language directives back. The launch controls expose the lane detector profile, Safety Mindset toggle, Ambient cruising toggle, persona picker, a one-click **Acceleration Mode** combo (Auto, CPU-only, CUDA, or the Quadro P520 compatibility path), and an **Enable Advisor** checkbox so you can silence narration when you need the lightest possible loop. Scroll a little farther to the **Lane Alignment & Warp** box to toggle the automatic horizon trim, dial in an extra manual tilt, and edit the four normalized trapezoid points so the overlay hugs whatever camera pitch or handlebar mount you are using.

4. **Use the *Media Test* tab** when you want to sanity-check the AI without riding the scooter. Point it at any road clip (`.mp4`, `.mov`, `.avi`, `.mkv`) or still photo (`.png`, `.jpg`, `.bmp`). The tab reuses your current launch settings, replays the media through the full pilot stack, and writes a new video with YOLO boxes, captions, and (when available) the translucent lane corridor burned into every frame. The overlay lane corridor follows the same fixed, downward-tilted calibration as the live feed so the lane band hugs the pavement instead of drifting toward the sky. Pick a folder or filename to save the overlay anywhere you want (or leave it blank to stick with `logs/media_tests/`). The preview panel streams the most recent overlay so you can watch the run while it renders, and a dedicated **Stop** button lets you cancel the export, discard the partial file, and switch clips instantly.

### Automated environment detection

The one-line launcher (`python scooter_app.py`) always performs a fresh scan before the GUI appears. The bootstrapper logs the detected environment and selects the matching dependency profile automatically:

- **Jetson (JetPack / L4T)** – Uses the Jetson-optimized dependency stack and NVIDIA's Python wheel index so PyTorch/OpenCV align with the JetPack image.
- **Ubuntu on WSL** – Uses the modern Linux stack but flags the runtime as WSL so you can enable GPU pass-through or USB forwarding as needed.
- **Native Ubuntu / desktop OS** – Classifies the compute tier (lightweight/standard/performance) and installs the corresponding CPU/GPU libraries.

If the host changes (e.g., you move the SD card to a different Jetson or upgrade your workstation), the cached profile is refreshed automatically at startup.

### Command-line workflow

If you prefer the CLI or need to run headless, you can still launch the pilot directly.

1. **Install dependencies** (Python 3.10+ recommended):

   The easiest path is to reuse the managed environment that the GUI bootstrapper creates.

   ```bash
   python setup_scroot.py --skip-models  # installs into .venv/
   source .venv/bin/activate             # or .venv\Scripts\activate on Windows
   ```

   If you prefer to drive pip manually, run the commands below *inside* that virtual environment so you avoid PEP 668 restrictions:

   ```bash
   python -m pip install --upgrade pip
   python -m pip install -r autonomy/requirements.txt
   ```

   On Jetson devices you may prefer the Jetson-specific OpenCV or PyTorch builds. Adjust the requirement accordingly if you already have hardware-accelerated packages installed.

2. **Start the SensorHub ROS pipeline** so `/sensor_hub/data` publishes fused frames and ranges:

   ```bash
   roslaunch sensor_interface sensor_interface_dynamic.launch
   ```

3. **Run the ai_input_bridge** so AutonomyPilot consumes `/sensor_hub/data` directly (and optionally mirrors runtime snapshots for the GUI):

   ```bash
   roslaunch scroot_bridge ai_input_bridge.launch               # launches the ROS node + AutonomyPilot
   # or, for tighter control:
   rosrun scroot_bridge ai_input_bridge.py _mirror_runtime_inputs:=false
   ```

   The bridge initializes the `ai_input_bridge` ROS node, converts each SensorHub message with `CvBridge`, feeds the fused frame and LiDAR ranges into the AI modules, mirrors `~/scroot/runtime_inputs/{camera.jpg,lidar.npy,sensor_meta.json}` when `mirror_runtime_inputs` stays `true`, and throttles logs to once per second.

4. **Optional CLI fallback** – if you are replaying recorded data or debugging offline, you can still launch the pilot via the convenience wrapper (which forwards to `autonomy_launcher.py` after dependency checks):

   ```bash
   python3 scroot/main.py --camera ~/scroot/runtime_inputs/camera.jpg --lidar ~/scroot/runtime_inputs/lidar.npy
   ```

   In offline mode the launcher reads the mirrored runtime snapshots, runs perception and the arbitration stack, and prints actuator commands together with the arbiter verdict and reasons:

   ```text
   time=3.42s steer=+0.120 throttle=0.280 brake=0.000 arbiter=AMEND reasons=vru_slow goal="drive 2 m forward"
   ```

   Press `q` in the visualization window or send `Ctrl+C` to exit.

5. **One-command tmux bring-up** – when you are ready to boot the entire stack (Sensor Interface, visualizer, dual YOLO, AI bridge, and GUI) from a clean slate, run the helper script from the repository root:

   ```bash
   cd ~/scroot
   ./scroot-Stable/run_full_stack.sh
   ```

   The script (a) executes the `catkin_ws/kill_sensors.sh` cleanup routine, (b) rebuilds the catkin workspace, and (c) spawns a `tmux` session named `scroot_stack` with five windows:

   | Window | Command |
   | --- | --- |
   | Sensors | `./kill_sensors.sh && catkin_make && source devel/setup.bash && roslaunch sensor_interface sensor_interface_dynamic.launch` |
   | Visualizer | `source devel/setup.bash && rosrun sensor_interface visualize_sensor_hub.py` |
   | DualYOLO | `source devel/setup.bash && rosrun scooter_control dual_yolo_node.py` |
   | AI Bridge | `source devel/setup.bash && rosrun scroot_bridge ai_input_bridge.py` |
   | GUI | `python3 scooter_app.py` (with `.venv` auto-activation when available or a custom `$SCROOT_VENV`) |

   Attach to the tmux session (the script does this automatically) to monitor logs per window. Exit with `Ctrl+C` inside each pane followed by `tmux detach`/`Ctrl+B d`.

## ROS Sensor Interface + Launch Files

The repository now mirrors the entire ROS1 side of the integration under `catkin_ws/src/` so a Jetson Orin can rebuild the topology exactly as shown in the reference `rqt_graph` screenshots.

1. **Build the workspace** (once per update):

   ```bash
   cd ~/scroot/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **Bring the sensor interface online** – `sensor_interface/scripts/sensor_interface_node.py` auto-detects `/usb_cam/image_raw` + `/scan`, logs “Topics detected…” followed by “Sensor Interface Node initialized successfully.” Use the provided launch file to start the entire hardware chain:

   ```bash
   roslaunch sensor_interface sensor_interface_dynamic.launch
   ```

   This boots `usb_cam`, `rplidar_ros` (baud `1000000` on `/dev/ttyUSB0`), and the SensorHub publisher.

3. **Run the dual TensorRT YOLO node** – the `scooter_control` package exposes `dual_yolo_node.py` plus `yolo.launch` so you can feed `/AI/annotated_image` and `/AI/detections` to visualization tools without touching the AI pipeline:

   ```bash
   roslaunch scooter_control yolo.launch
   # or tweak engines/thresholds:
   rosrun scooter_control dual_yolo_node.py _primary_engine:=~/models/yolov8n_fp16.engine
   ```

4. **Start Abdallah’s bridge** – the `scroot_bridge` package wraps `~/scroot/bridge/ai_input_bridge.py` so ROS launch files can manage it. Launch it on its own or as part of the combined bring-up:

   ```bash
   roslaunch scroot_bridge ai_input_bridge.launch               # only the bridge
   roslaunch scroot_bridge full_system.launch                  # usb_cam + RPLIDAR + SensorHub + YOLO + bridge
   ```

5. **CLI equivalents for component testing** – if you prefer explicit `rosrun` commands, the expected sequence is:

   ```bash
   roscore
   rosrun usb_cam usb_cam_node
   rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=1000000
   rosrun sensor_interface sensor_interface_node.py
   rosrun scooter_control dual_yolo_node.py
   rosrun scroot_bridge ai_input_bridge.py
   ```

   Once `/sensor_hub/data` is flowing you can rely on `ai_input_bridge` to keep the AI running. The optional runtime mirror keeps `~/scroot/runtime_inputs/camera.jpg`, `lidar.npy`, and `sensor_meta.json` fresh for the GUI and offline CLI runs, and `rqt_graph` now shows the canonical flow: `/usb_cam` + `/scan` → `sensor_interface_node` → `/sensor_hub/data` → `ai_input_bridge` (feeding AutonomyPilot).

## Configuration Options

The launcher accepts several runtime flags:

| Flag | Description | Default |
| --- | --- | --- |
| `--camera` | Camera image path for offline/GUI runs (live ROS mode ignores this) | `~/scroot/runtime_inputs/camera.jpg` |
| `--width` / `--height` | Capture resolution | `1280x720` |
| `--fps` | Target frame rate | `30` |
| `--model` | YOLO model file | `yolov8n.pt` |
| `--confidence` | Detection confidence threshold | `0.3` |
| `--iou` | Detection IoU threshold | `0.4` |
| `--device {auto,cpu,cuda,quadro_p520}` | Choose auto-detect, force CPU-only, force CUDA, or engage the Quadro P520 compatibility mode (locks YOLO to `cuda:0`). | `auto` |
| `--visualize` | Enable on-screen overlays | Disabled |
| `--log-dir` | Directory for visualizations and state dumps | `logs/` |
| `--command` | Initial natural-language goal (e.g. `"drive to the park"`) | None |
| `--command-file` | Path to a UTF-8 text file that can be updated with new commands | None |
| `--lane-sensitivity {precision,balanced,aggressive}` | Choose the lane detector’s confidence/smoothing profile | `balanced` |
| `--command-state` | JSON file capturing the parsed command | `logs/command_state.json` |
| `--safety-mindset {on,off}` | Enable speed/clearance caps from mindset profiles | `off` |
| `--ambient {on,off}` | Enable ambient cruising (slow roll only when the arbiter approves) | `on` |
| `--persona {calm_safe,smart_scout,playful}` | Pick narration tone for the riding companion | `calm_safe` |
| `--advisor {on,off}` | Enable or disable the riding companion advisor | `on` |
| `--vehicle-description` | Human-readable name used in logs and narration context | `Scooter` |
| `--vehicle-width` / `--vehicle-length` / `--vehicle-height` | Physical dimensions in meters | `0.65 / 1.2 / 1.2` |
| `--clearance-margin` | Additional lateral buffer per side (meters) | `0.2` |
| `--calibration-distance` | Distance in meters used to anchor pixel→meter scaling | `2.0` |
| `--calibration-pixels` | Observed pixel width of the vehicle at the calibration distance | `220.0` |
| `--lidar` | Path to the LiDAR `.npy` file mirrored by `ai_input_bridge` | `~/scroot/runtime_inputs/lidar.npy` |

## How It Works

1. **AI Input Bridge** (`bridge/ai_input_bridge.py`) subscribes to `/sensor_hub/data`, converts each SensorHub packet into `SensorSample` + `LidarSnapshot` objects, and calls `AutonomyPilot.process_sample()` so the navigator, arbiter, and controller always operate on the real-time ROS feed.
2. **CameraSensor** (`autonomy/sensors/camera.py`) remains available for offline CLI runs and the GUI, continuously polling `runtime_inputs/camera.jpg` when the bridge mirrors snapshots.
3. **LidarSensor** (`autonomy/sensors/lidar.py`) reads `runtime_inputs/lidar.npy` + `sensor_meta.json` for offline playback, ensuring each `LidarSnapshot` includes ranges, scan angles, SensorHub timestamp, IMU vector, and ultrasonic distance for planners, telemetry, and the GUI.
4. **ObjectDetector** (`autonomy/perception/object_detection.py`) identifies obstacles using YOLO and returns bounding boxes. The detector now honors `--device`/Acceleration Mode, so you can run entirely on CPU.
4. **LaneDetector** (`autonomy/perception/lane_detection.py`) equalizes lighting, runs Sobel/Canny filters, applies a pre-tuned downward-tilted warp, warps the roadway into a cached bird’s-eye view, fits lane lines, and reports curvature/offset values that mirror openpilot’s lateral planner. The results are used strictly for visualization and logging.
5. **CommandInterface** (`autonomy/ai/command_interface.py`) parses operator phrases and exposes structured goals.
6. **Navigator** (`autonomy/planning/navigator.py`) blends obstacle density and the active goal to produce a steering bias, target speed, and goal context—without consuming lane geometry so the AI stays in charge of motion even when no lines are visible.
7. **Controller** (`autonomy/control/controller.py`) converts navigation decisions into smoothed actuator commands, prioritizing braking when hazards or enforced stops occur.
8. **ControlArbiter** (`autonomy/control/arbitration.py`) fuses the Pilot proposal with the deterministic safety arbiter verdict, Safety Mindset caps, and SafetyGate clamps to publish the final actuator command.
9. **Riding Companion** (`autonomy/control/companion.py`) narrates each arbiter decision when enabled, while **TelemetryLogger** (`autonomy/control/telemetry.py`) records JSONL logs for auditing.

### Lane detection and openpilot inspiration

The upgraded lane detector still mirrors openpilot’s lateral planner but now layers several lightweight enhancements so it performs well on CPU-only laptops and Jetson Orin dev kits:

1. **Contrast-limited equalization + multi-color masks** – The lightness channel runs through CLAHE, HSV value/saturation masks keep dark asphalt usable, and both HLS yellow ranges plus LAB “b” thresholds highlight painted or thermoplastic tape even when it fades.
2. **Gradient + adaptive edges** – Sobel derivatives, Canny edges, and an adaptive threshold fuse into a single binary mask so faint grooves, cat-eyes, or green bike lanes still count toward the search histogram.
3. **Auto-trimmed bird’s-eye warp with manual overrides** – The GUI lets you edit the four normalized warp points and apply a manual tilt, while the detector adds a smoothed auto horizon trim (bounded to ±0.08 of the frame) so uploads from new scooters stay glued to the pavement.
4. **Cached perspective transforms** – Homographies are cached per resolution so each frame jumps straight into the top-down view without recomputing matrices.
5. **Sliding-window + previous-fit tracker** – The detector gathers lane pixels with the classic openpilot window search and falls back to a “search around poly” mode when lines are faint, smoothing polynomial fits frame-to-frame.
6. **Reprojection + overlay** – The left/right rails and the center corridor are projected back on top of the live feed, filled with a translucent highlight, and echoed in a mini bird’s-eye inset so you can see what the detector thinks at a glance.

From those fits we derive lane width, curvature, and the scooter’s lateral offset in meters. Those numbers feed the overlay (so you can see the translucent corridor, rails, and bird’s-eye inset) and telemetry only—the navigator, controller, and arbiter never touch them. If the detector does not see a lane, the overlay simply stays hidden and the AI keeps following YOLO detections plus the active goal.

## Command Interface Tips

- Update the file passed with `--command-file` to issue live instructions. Each save triggers a re-parse.
- Supported phrases include:
  - `drive 5 m forward`
  - `drive to the loading dock`
  - `turn around`
  - `turn left`
  - `stop`
- Any unrecognized text still reaches the navigator/arbiter pipeline so free-form goals remain audible and logged.

## Safety Arbiter & Safety Mindset

The deterministic safety arbiter inspects each control tick and chooses one of three verdicts:

- **ALLOW** – Pilot command passes through (after SafetyGate range checks). Mindset caps still limit throttle.
- **AMEND** – Arbiter publishes a safer command (e.g., steer toward the bike lane, reduce speed near pedestrians).
- **BLOCK** – Immediate fail-stop (`steer=0`, `throttle=0`, `brake=1`). The scooter holds the stop until risk stays low for the configured debounce window *and* a fresh, safe proposal arrives.

Key triggers:

- **Fail-stop** when time-to-collision drops below the configured `ttc_block_s`, hazard level peaks, sensor confidence collapses, or the vehicle envelope would collide with nearby obstacles/curbs.
- **Takeover** (AMEND) when vulnerable road users are close, the Safety Mindset cap is exceeded, or lateral clearance tightens and the arbiter nudges away from the hazard.
- **Timeout** safeguards: if the arbiter exceeds its time budget, the previous verdict persists for one tick, then the system defaults to BLOCK.

The optional Safety Mindset applies contextual caps before arbitration. Profiles such as `cautious_pedestrian`, `low_visibility`, and `worst_case_child` adjust maximum speed and minimum clearance. Uncertainty bias further reduces caps whenever perception confidence is degraded.

Ambient mode gates motion when no explicit goal is set—movement occurs only when the arbiter returns ALLOW/AMEND, and throttle is limited to a gentle cruise. The riding companion narrates decisions every ≥2 s (e.g., “Fail-stop: pedestrian crossing ahead”).

- **normal** – Balanced thresholds. The arbiter AMENDs before BLOCKing when possible, and allows one tick of timeout grace.
- **strict** – Conservative behavior. Higher confidence required to ALLOW, faster BLOCK response on TTC, longer debounce, and zero timeout grace (defaults to BLOCK on overruns).

### Demo Scenarios

| Scenario | Expected Behavior |
| --- | --- |
| Obstacle 1.5 m ahead at cruising speed | Arbiter emits `BLOCK` within one tick, SafetyGate holds full brake until obstacle clears + debounce. Logged with reason `ttc_low` and incident entry. |
| Forced arbiter timeout | Latency budget exceeded → previous verdict reused for ≤1 tick, then `BLOCK` with reason `timeout`. |
| Safety Mindset toggle | With mindset OFF, throttle follows Pilot target. Turning mindset ON lowers `caps_speed` and logs the active profile. |
| Companion narration | Messages such as “Fail-stop: pedestrian crossing ahead” appear in logs no more than once every 2 s. |

### Telemetry & Incident Logging

- `logs/telemetry.jsonl` records every tick with arbiter decision, reason tags, latency, proposed vs final command, caps, and navigation sub-goal state.
- `logs/incidents.jsonl` captures each AMEND/BLOCK event with a reference to 5 s pre/post clips (populate the clip fields if you archive video). Use these logs to audit fail-stop coverage and arbiter timing.
- Vehicle envelope metadata (lane width estimate, left/right clearance, required clearance) is emitted with each tick so you can audit why the arbiter held, amended, or released control.

## Extending the System

- **Additional sensors:** Feed their obstacle cues into `Navigator.plan` by augmenting the occupancy map.
- **Custom models:** Provide a different YOLO checkpoint via `--model` or tune `--lane-sensitivity` to match your optics and road surface.
- **Vehicle integration:** Map the normalized actuator outputs to your scooter's control API. For example, scale `steer` to handlebar servo angles and translate `throttle`/`brake` to PWM duty cycles.

## Safety Notice

This codebase is intended for research and prototyping. Always test in a controlled environment, keep a human operator ready to take over, and comply with local regulations for sidewalk and bike-lane operation.
