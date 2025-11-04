# Autonomous Scooter Pilot

This package contains a self-contained autonomous driving stack tailored for lightweight vehicles such as scooters. The system is designed to run on NVIDIA Jetson devices as well as standard Ubuntu 24.04 laptops, using a single USB camera for perception. It exposes raw actuator values (`steer`, `throttle`, `brake`) that you can feed directly into your hardware interface. All commands below assume you are inside the `scroot` directory.

## Features

- Plug-and-play sensor registry with camera and LiDAR adapters; swap in alternate perception sensors without touching the pilot.
- Real-time object detection powered by Ultralytics YOLO models (default: `yolov8n.pt`).
- Hybrid navigation that fuses obstacle density analysis with natural-language operator goals.
- Mode manager that coordinates `idle`, `cruise`, `summon`, and `emergency_stop` behavior for rider cruise control and remote summon workflows.
- Guardian safety layer that monitors hazard levels, ingests LiDAR-based proximity checks, enforces slowdowns/hard stops, and surfaces alerts for accessibility cues.
- Lightweight localization tracker and summon planner that modulate behavior based on multi-sensor confidence.
- Optional multimodal advisor (BLIP + FLAN-T5) that captions the scene and issues traffic-law-compliant directives ("use the bike lane", "yield", "slow down").
- Command parser that understands phrases such as "drive to the plaza", "drive 2 m forward", or "turn right" and feeds them into the navigator.
- Structured state export (`logs/command_state.json`, `logs/advisor_state.json`) plus MCU-friendly CSV logging for telemetry, remote supervision, or UI dashboards.

## Quick Start

1. **Install dependencies** (Python 3.10+ recommended):

   ```bash
   python -m pip install --upgrade pip
   python -m pip install -r autonomy/requirements.txt
   ```

   On Jetson devices you may prefer the Jetson-specific OpenCV or PyTorch builds. Adjust the requirement accordingly if you already have hardware-accelerated packages installed.

2. **Connect your camera** via USB and determine its index (usually `0`).

3. **Launch the pilot** using the unified launcher:

   ```bash
   python autonomy_launcher.py --camera 0 --visualize --command "drive 2 m forward"
   ```

   The launcher checks dependencies, starts the camera, runs perception and control, and prints actuator commands together with the advisor verdict:

   ```text
   time=3.42s steer=+0.120 throttle=0.320 brake=0.000 directive="stay in bike lane" goal="drive 2 m forward"
   ```

   Press `q` in the visualization window or send `Ctrl+C` to exit.

   To record an annotated run without displaying a window, add `--video-output logs/run.mp4`; include `--visualize` if you also want the live preview.
   The bundled `sample_video.mp4` is only a placeholder clip—replace it with your own road footage or pass a live camera index (such as `--camera 0`) to exercise the stack in a driving scenario.

## Configuration Options

The launcher accepts several runtime flags:

| Flag | Description | Default |
| --- | --- | --- |
| `--camera` | Camera index or video path | `0` |
| `--sensor` | Sensor adapter key registered with the plug-and-play registry | `camera` |
| `--sensor-arg` | Additional sensor arguments (`KEY=VALUE`, repeatable) | None |
| `--sensors-config` | JSON file defining one or more sensors via adapter/name/args | None |
| `--width` / `--height` | Capture resolution | `1280x720` |
| `--fps` | Target frame rate | `30` |
| `--model` | YOLO model file | `yolov8n.pt` |
| `--confidence` | Detection confidence threshold | `0.3` |
| `--iou` | Detection IoU threshold | `0.4` |
| `--visualize` | Enable on-screen overlays | Disabled |
| `--video-output` | Write an annotated MP4 with detections and control overlays | None |
| `--mode` | Initial operating mode (`idle`, `cruise`, `summon`) | `cruise` |
| `--cruise-speed` | Max speed (m/s) allowed in cruise mode | `4.5` |
| `--summon-speed` | Max speed (m/s) allowed in summon mode | `3.0` |
| `--guardian-slowdown` | Hazard threshold that triggers proactive slowdowns | `0.5` |
| `--guardian-emergency` | Hazard threshold that forces an emergency stop | `0.85` |
| `--guardian-min-speed-scale` | Minimum speed multiplier during guardian slowdowns | `0.2` |
| `--guardian-lidar-slowdown` | LiDAR distance (m) that triggers proactive slowdowns | `2.5` |
| `--guardian-lidar-stop` | LiDAR distance (m) that forces an emergency stop | `1.0` |
| `--command-log` | CSV file capturing actuator commands mapped to MCU units | None |
| `--log-dir` | Directory for visualizations and state dumps | `logs/` |
| `--command` | Initial natural-language goal (e.g. `"drive to the park"`) | None |
| `--command-file` | Path to a UTF-8 text file that can be updated with new commands | None |
| `--disable-advisor` | Skip loading the multimodal advisor | Enabled |
| `--advisor-image-model` | Hugging Face name for the BLIP image encoder | `Salesforce/blip-image-captioning-base` |
| `--advisor-language-model` | Hugging Face name for the language model | `google/flan-t5-small` |
| `--advisor-device` | Force the device used by the advisor (`cpu`, `cuda`, etc.) | Auto-detect |
| `--advisor-state` | JSON file capturing the latest advisor directive | `logs/advisor_state.json` |
| `--command-state` | JSON file capturing the parsed command | `logs/command_state.json` |
| `--steer-center` | Steering servo center in MCU units (e.g., microseconds) | `1500.0` |
| `--steer-range` | Steering delta applied for `steer=-1/+1` | `400.0` |
| `--throttle-scale` | Scale factor applied to throttle before sending to MCU | `255.0` |
| `--throttle-offset` | Offset added to throttle output | `0.0` |
| `--brake-scale` | Scale factor applied to brake before sending to MCU | `255.0` |
| `--brake-offset` | Offset added to brake output | `0.0` |

## MCU Integration Workflow

- The pilot prints normalized commands (`steer`, `throttle`, `brake`) alongside MCU-ready values (`steer_hw`, `throttle_hw`, `brake_hw`) each cycle. Adjust the mapping to your hardware with `--steer-center`, `--steer-range`, `--throttle-scale`, `--throttle-offset`, `--brake-scale`, and `--brake-offset`.
- Provide `--command-log logs/commands.csv` (or another path) to capture every update in a CSV file you can replay or stream to firmware.
- To keep a live dashboard synchronized, read `logs/command_state.json`, which updates with the latest parsed operator goal, and `logs/advisor_state.json` for advisory directives.
- Guardian alerts (e.g., `hazard_slowdown`, `hazard_emergency`) show up in both stdout and the command CSV so accessibility UIs can trigger audio or haptic cues.

## Sensor Interface

- Load multiple sensors by supplying a JSON spec: `python autonomy_launcher.py --sensors-config configs/sensors.json`. Each entry should provide an `adapter`, optional `name`, and `args` map. Example:

  ```json
  [
    {"adapter": "camera", "name": "front_cam", "args": {"source": 0, "width": 1280, "height": 720}},
    {"adapter": "lidar", "name": "roof_lidar", "args": {"port": "/dev/ttyUSB0", "baud": 115200}}
  ]
  ```

- The first entry designates the primary sensor whose stream feeds the perception stack; the rest remain available through the `SensorManager` for guardian or localization consumers.
- Use `--sensor` and `--sensor-arg` for quick single-sensor experiments (these flags populate the same spec list under the hood).
- To add a new adapter, implement a `StreamingSensor` subclass in `autonomy/sensors/`, register it with `global_sensor_registry`, and optionally consume its `SensorSample` outputs within the guardian, navigator, or downstream components.
- The built-in `LiDARSensor` can replay newline-delimited JSON (`{"ranges": [...], "angle_increment": ...}`) or simple CSV scans; pair it with the guardian's LiDAR thresholds to tighten proximity handling.

## Operating Modes & Guardian

- Select the initial mode with `--mode` (`idle`, `cruise`, or `summon`). Cruise holds rider-selected speed, while summon caps velocity aggressively for autonomous “come-to-me” behavior.
- `--cruise-speed` and `--summon-speed` set per-mode speed ceilings, letting you tune for sidewalk, bike-lane, or campus policies.
- The guardian layer enforces slowdowns once hazard scores exceed `--guardian-slowdown` and commands full stops beyond `--guardian-emergency`. Alerts surface through stdout, CSV logging, and the advisor JSON for accessibility tooling.
- When hazards clear, the mode manager automatically resumes the requested mode so the scooter returns to normal cruise or summon flow.

## How It Works

1. **Sensor Registry + CameraSensor** (`autonomy/sensors/__init__.py`, `autonomy/sensors/camera.py`) provide a plug-and-play interface for video and future sensors.
2. **ObjectDetector** (`autonomy/perception/object_detection.py`) identifies obstacles using YOLO and returns bounding boxes.
3. **CommandInterface** (`autonomy/ai/command_interface.py`) parses operator phrases and exposes structured goals.
4. **Navigator** (`autonomy/planning/navigator.py`) evaluates obstacle density and the active goal to produce a steering bias, target speed, and goal context.
5. **Guardian** (`autonomy/safety/guardian.py`) watches hazard levels and priority objects, emitting slowdowns, emergency stops, and accessibility alerts.
6. **ModeManager** (`autonomy/core/mode_manager.py`) enforces cruise, summon, idle, and emergency modes before commands reach the controller.
7. **SituationalAdvisor** (`autonomy/ai/advisor.py`) captions the scene, blends detection metadata with the goal, and emits traffic-law-compliant directives or enforced stops.
8. **Controller** (`autonomy/control/controller.py`) converts navigation decisions into smoothed actuator commands, prioritizing braking when hazards or enforced stops occur.
9. **AutonomyPilot** (`autonomy/pilot.py`) orchestrates these components, exports telemetry, writes annotated video/logs, and prints raw actuator values for MCU integration.

## Command Interface Tips

- Update the file passed with `--command-file` to issue live instructions. Each save triggers a re-parse.
- Supported phrases include:
  - `drive 5 m forward`
  - `drive to the loading dock`
  - `turn around`
  - `turn left`
  - `stop`
- Any unrecognized text is still passed to the advisor so it can reason about free-form goals.

## Safety Advisor Behavior

The advisor enforces conservative behavior:

- Stops when YOLO detects `stop sign`, `traffic light`, `person`, or `bicycle` directly ahead.
- Prioritizes sidewalks or bike lanes in its textual guidance.
- Raises braking priority when the navigator reports a high hazard score (>0.85 by default).
- Exports human-readable directives and scene captions for auditing.

## Jetson Deployment Notes

- Install NVIDIA's CUDA-accelerated builds of PyTorch, TorchVision, and OpenCV before running `pip install -r autonomy/requirements.txt`; the pip resolver will reuse the preinstalled wheels.
- Use GStreamer-backed camera sources when possible. For example: `python autonomy_launcher.py --sensor camera --sensor-arg source='nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink'`.
- Set `--advisor-device cuda` to keep multimodal models on the GPU, or disable the advisor on resource-constrained SKUs with `--disable-advisor`.
- Leverage the MCU mapping options plus `--command-log` to stream PWM/GPIO commands to carrier-board microcontrollers over UART or CAN.
- For LiDAR integration, install the appropriate vendor SDK, point the `LiDARSensor` adapter at the device (`--sensors-config`), and ensure the user running the pilot has access to the underlying serial/ethernet interface.

## Extending the System

- **Additional sensors:** Implement a `StreamingSensor` adapter in `autonomy/sensors/` and register it so the pilot can auto-discover new cameras, depth sensors, or ultrasonic arrays.
- **Guardian tuning:** Adjust `GuardianConfig` or supply custom guardian logic to incorporate lane semantics, geofences, and accessibility-specific cues.
- **Vehicle integration:** Map the normalized actuator outputs to your scooter's control API. For example, scale `steer` to handlebar servo angles and translate `throttle`/`brake` to PWM duty cycles; use the MCU command log to validate mappings.

## Safety Notice

This codebase is intended for research and prototyping. Always test in a controlled environment, keep a human operator ready to take over, and comply with local regulations for sidewalk and bike-lane operation.
