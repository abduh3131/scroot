# Autonomous Scooter Pilot

This package contains a self-contained autonomous driving stack tailored for lightweight vehicles such as scooters. The system is designed to run on NVIDIA Jetson devices as well as standard Ubuntu 24.04 laptops, using a single USB camera for perception. It exposes raw actuator values (`steer`, `throttle`, `brake`) that you can feed directly into your hardware interface. All commands below assume you are inside the `scroot` directory.

## Features

- Camera ingestion pipeline compatible with any USB camera supported by OpenCV.
- Real-time object detection powered by Ultralytics YOLO models (default: `yolov8n.pt`).
- Hybrid navigation that fuses obstacle density analysis with natural-language operator goals.
- Safety-aware control layer that outputs normalized actuator commands, brakes for hazards, and honors enforced stops.
- OpenCV-based lane detector with perspective warps, sliding-window fits, and curvature/offset estimates inspired by openpilot’s lateral planner.
- Lane-aware navigator that blends those lane offsets back into the steering bias so the pilot recenters itself instead of drifting across bike lanes or shoulders.
- Command parser that understands phrases such as "drive to the plaza", "drive 2 m forward", or "turn right" and feeds them into the navigator.
- Structured state export (`logs/command_state.json`) for telemetry, remote supervision, or UI dashboards.
- Modular design that allows future sensors (ultrasonic, LiDAR, depth) to feed into the navigator without architectural changes.
- CPU/GPU acceleration toggle plus an advisor switch so you can run the pilot entirely on CPU and silence narration when you need the leanest loop.

## Quick Start

1. **Run the bootstrapper** (creates a virtual environment and downloads models):

   ```bash
   python setup_scroot.py
   ```

   The script installs all Python dependencies into `.venv/`, caches the default YOLOv8 checkpoint under `models/`, and prepares runtime directories. Add `--skip-models` if you want to reuse previously downloaded weights.

2. **Activate the environment** created by the bootstrapper:

   ```bash
   source .venv/bin/activate
   ```

3. **Connect your camera** via USB and determine its index (usually `0`).

4. **Launch the pilot** using the unified launcher:

   ```bash
   python autonomy_launcher.py --camera 0 --visualize --command "drive 2 m forward"
   ```

   The launcher checks dependencies, starts the camera, runs perception and control, and prints actuator commands together with the arbiter verdict:

   ```text
   time=3.42s steer=+0.120 throttle=0.320 brake=0.000 directive="stay in bike lane" goal="drive 2 m forward"
   ```

   Press `q` in the visualization window or send `Ctrl+C` to exit.

### Bootstrap Script Options

`setup_scroot.py` accepts a few helpful flags:

| Flag | Purpose |
| --- | --- |
| `--venv PATH` | Place the virtual environment in a custom directory (default: `./.venv`). |
| `--models-dir PATH` | Choose where YOLO weights are cached (default: `./models`). |
| `--skip-models` | Install Python packages but reuse previously downloaded model weights. |
| `--upgrade` | Recreate the virtual environment from scratch before installing dependencies. |

## Configuration Options

The launcher accepts several runtime flags:

| Flag | Description | Default |
| --- | --- | --- |
| `--camera` | Camera index or video path | `0` |
| `--width` / `--height` | Capture resolution | `1280x720` |
| `--fps` | Target frame rate | `30` |
| `--model` | YOLO model file | `yolov8n.pt` |
| `--confidence` | Detection confidence threshold | `0.3` |
| `--iou` | Detection IoU threshold | `0.4` |
| `--device {auto,cpu,cuda}` | Force YOLO to run on CPU, CUDA, or auto-detect | `auto` |
| `--visualize` | Enable on-screen overlays | Disabled |
| `--log-dir` | Directory for visualizations and state dumps | `logs/` |
| `--command` | Initial natural-language goal (e.g. `"drive to the park"`) | None |
| `--command-file` | Path to a UTF-8 text file that can be updated with new commands | None |
| `--lane-sensitivity {precision,balanced,aggressive}` | Tune lane detector smoothing/confidence | `balanced` |
| `--command-state` | JSON file capturing the parsed command | `logs/command_state.json` |
| `--advisor {on,off}` | Enable or disable the riding companion advisor | `on` |

## How It Works

1. **CameraSensor** (`autonomy/sensors/camera.py`) continuously streams frames.
2. **ObjectDetector** (`autonomy/perception/object_detection.py`) identifies obstacles using YOLO and returns bounding boxes (respecting the new `--device`/Acceleration Mode setting so CPU-only runs avoid CUDA errors).
3. **LaneDetector** (`autonomy/perception/lane_detection.py`) warps the road surface into a bird’s-eye view, runs sliding windows to fit lane lines, and publishes curvature, width, and lateral offset estimates.
4. **CommandInterface** (`autonomy/ai/command_interface.py`) parses operator phrases and exposes structured goals.
5. **Navigator** (`autonomy/planning/navigator.py`) blends obstacle density, lane geometry, and the active goal to produce a steering bias, target speed, and goal context; lane offsets now actively nudge the steering bias back toward the center of any detected lane.
6. **Control Arbiter** (`autonomy/control/arbitration.py`) enforces deterministic ALLOW/AMEND/BLOCK decisions based on hazard level, lane confidence, vulnerable road users, and operator caps.
7. **Controller** (`autonomy/control/controller.py`) converts navigation decisions into smoothed actuator commands, prioritizing braking when hazards or enforced stops occur.
8. **AutonomyPilot** (`autonomy/pilot.py`) orchestrates these components, exports telemetry, and prints raw actuator values for integration with your vehicle controller; the riding companion advisor narrates arbiter decisions only when enabled.

## Command Interface Tips

- Update the file passed with `--command-file` to issue live instructions. Each save triggers a re-parse.
- Supported phrases include:
  - `drive 5 m forward`
  - `drive to the loading dock`
  - `turn around`
  - `turn left`
  - `stop`
- Any unrecognized text still reaches the navigator, which treats it as a free-text goal for the arbiter to evaluate.

## Safety Arbiter Behavior

The rule-based arbiter enforces conservative behavior:

- Blocks motion when the predicted time-to-collision drops below the configured threshold or when the vehicle envelope is about to clip a lane boundary.
- Biases the steering command away from clutter whenever the lane detector reports an offset or low confidence.
- Applies AMEND decisions that cap throttle around vulnerable road users or when the safety mindset requests lower clearances.
- Logs every ALLOW/AMEND/BLOCK verdict with explicit reason tags for auditing.

## Extending the System

- **Additional sensors:** Feed their obstacle cues into `Navigator.plan` by augmenting the occupancy map.
- **Custom models:** Provide a different YOLO checkpoint via `--model` or tweak the `--lane-sensitivity` flag to match your camera placement and road texture.
- **Vehicle integration:** Map the normalized actuator outputs to your scooter's control API. For example, scale `steer` to handlebar servo angles and translate `throttle`/`brake` to PWM duty cycles.

## Safety Notice

This codebase is intended for research and prototyping. Always test in a controlled environment, keep a human operator ready to take over, and comply with local regulations for sidewalk and bike-lane operation.
