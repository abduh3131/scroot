# Autonomous Scooter Pilot

This package contains a self-contained autonomous driving stack tailored for lightweight vehicles such as scooters. The system is designed to run on NVIDIA Jetson devices as well as standard Ubuntu 24.04 laptops, using a single USB camera for perception. It exposes raw actuator values (`steer`, `throttle`, `brake`) that you can feed directly into your hardware interface. All commands below assume you are inside the `scroot` directory.

## Features

- Camera ingestion pipeline compatible with any USB camera supported by OpenCV.
- Real-time object detection powered by Ultralytics YOLO models (default: `yolov8n.pt`).
- Hybrid navigation that fuses obstacle density analysis with natural-language operator goals.
- Layered arbitration stack (Navigator → Arbiter → SafetyGate) that can ALLOW, AMEND, or BLOCK commands every control tick.
- Optional safety mindset profiles that cap maximum speed / clearance and adapt to lane uncertainty or sensor degradation.
- Vehicle envelope modeling with user-defined dimensions and camera calibration to keep commands within real clearance limits.
- Riding companion narration that explains each arbiter decision without blocking the control loop.
- Advanced bird’s-eye lane detector (mirrors openpilot’s lateral planner) that equalizes lighting, auto-calibrates the perspective warp to whatever camera you attach, and feeds curvature/offset data straight back into the pilot.
- Lane-aware navigator that blends those lane offsets back into the steering bias so the pilot recenters inside bike lanes or shoulders automatically.
- CPU/GPU acceleration toggle plus an advisor switch so you can run entirely on CPU hardware, enable a Quadro P520 compatibility mode, and silence narration when you need the lightest loop.
- Live launch dashboard with camera feed, translucent lane-corridor overlays, throttle/brake gauges, actuator readouts, and lane/arbiter status plus a bird’s-eye mini-map.
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

3. **Open the *Launch* tab** to start the autonomy stack. Choose your camera index, tweak the frame size or FPS if needed, and press **Start Pilot**. Logs from the pilot appear in real time inside the GUI. The live feed shows the current frame, projected trajectory lines, throttle/brake gauges, actuator readouts, the reconstructed lane corridor, and safety-arbiter verdicts. Use the messaging panel to monitor ALLOW/AMEND/BLOCK decisions and send natural-language directives back. The launch controls expose the lane detector profile, Safety Mindset toggle, Ambient cruising toggle, persona picker, a one-click **Acceleration Mode** combo (Auto, CPU-only, CUDA, or the Quadro P520 compatibility path), and an **Enable Advisor** checkbox so you can silence narration when you need the lightest possible loop.

4. **Use the *Media Test* tab** when you want to sanity-check the AI without riding the scooter. Point it at any road clip (`.mp4`, `.mov`, `.avi`, `.mkv`) or still photo (`.png`, `.jpg`, `.bmp`). The tab reuses your current launch settings, replays the media through the full pilot stack, and writes a new video with the steering vector, throttle/brake bars, lane clearance gauges, directives, and captions burned into every frame. The overlay lane corridor is now auto-aligned using the same adaptive calibration as the live feed, so clips recorded with different camera heights still render correctly. Pick a folder or filename to save the overlay anywhere you want (or leave it blank to stick with `logs/media_tests/`). The preview panel streams the most recent overlay so you can watch the run while it renders, and a dedicated **Stop** button lets you cancel the export, discard the partial file, and switch clips instantly.

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

2. **Connect your camera** via USB and determine its index (usually `0`).

3. **Launch the pilot** using the unified launcher:

   ```bash
   python autonomy_launcher.py --camera 0 --lane-sensitivity precision --safety-mindset on --ambient on
   ```

   The launcher checks dependencies, starts the camera, runs perception and the arbitration stack, and prints actuator commands together with the arbiter verdict and reasons:

   ```text
   time=3.42s steer=+0.120 throttle=0.280 brake=0.000 arbiter=AMEND reasons=lane_bias_right,vru_slow goal="drive 2 m forward"
   ```

   Press `q` in the visualization window or send `Ctrl+C` to exit.

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

## How It Works

1. **CameraSensor** (`autonomy/sensors/camera.py`) continuously streams frames.
2. **ObjectDetector** (`autonomy/perception/object_detection.py`) identifies obstacles using YOLO and returns bounding boxes. The detector now honors `--device`/Acceleration Mode, so you can run entirely on CPU.
3. **LaneDetector** (`autonomy/perception/lane_detection.py`) equalizes lighting, runs Sobel/Canny filters, auto-tunes the horizon/warp for the current camera, warps the roadway into a cached bird’s-eye view, fits lane lines, and reports curvature/offset values that mirror openpilot’s lateral planner.
4. **CommandInterface** (`autonomy/ai/command_interface.py`) parses operator phrases and exposes structured goals.
5. **Navigator** (`autonomy/planning/navigator.py`) blends obstacle density, lane geometry, and the active goal to produce a steering bias, target speed, and goal context. Lane offsets directly nudge the steering bias back toward the center of any detected lane/shoulder.
6. **Controller** (`autonomy/control/controller.py`) converts navigation decisions into smoothed actuator commands, prioritizing braking when hazards or enforced stops occur.
7. **ControlArbiter** (`autonomy/control/arbitration.py`) fuses the Pilot proposal with the deterministic safety arbiter verdict, Safety Mindset caps, and SafetyGate clamps to publish the final actuator command.
8. **Riding Companion** (`autonomy/control/companion.py`) narrates each arbiter decision when enabled, while **TelemetryLogger** (`autonomy/control/telemetry.py`) records JSONL logs for auditing.

### Lane detection and openpilot inspiration

The upgraded lane detector still mirrors openpilot’s lateral planner but now layers several lightweight enhancements so it performs well on CPU-only laptops and Jetson Orin dev kits:

1. **Contrast-limited equalization + color masks** – The HLS lightness channel runs through CLAHE so worn paint and low-light clips remain visible before we threshold for white/yellow markings.
2. **Gradient + Canny edges** – Sobel derivatives and Canny edges catch faded lane tape that color alone would miss.
3. **Bird’s-eye warp cache** – Perspective matrices are cached per resolution so each frame jumps straight into the top-down view without re-solving homographies.
4. **Auto-calibrated horizon + warp** – A lightweight Hough transform finds the dominant left/right rails, recalculates the trapezoid used for the bird’s-eye transform, and smooths it over time so new cameras or mounting angles line up automatically.
5. **Sliding-window tracker** – The detector gathers lane pixels with the classic openpilot window search, smooths polynomial fits frame-to-frame, and rejects results that fall below the configured confidence.
6. **Reprojection + overlay** – The left/right rails and the center corridor are projected back on top of the live feed, filled with a translucent highlight, and echoed in a mini bird’s-eye inset so you can see what the detector thinks at a glance.

From those fits we derive lane width, curvature, and the scooter’s lateral offset in meters. The navigator feeds those offsets back into the steering bias so the pilot recenters inside any detected lane, while the arbiter tags AMEND/BLOCK events with `lane_bias_right`, `lane_too_narrow`, or `unknown_lane` when confidence drops. Lane-induced hazards are now capped so the pilot eases off the throttle without over-braking when the overlay needs a moment to re-center.

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

- **Fail-stop** when time-to-collision drops below the configured `ttc_block_s`, hazard level peaks, lane mismatch is detected, sensor confidence collapses, or the vehicle envelope would collide with nearby obstacles/curbs.
- **Takeover** (AMEND) when a safer lateral bias exists, vulnerable road users are close, the Safety Mindset cap is exceeded, or lateral clearance tightens and the arbiter nudges away from the hazard.
- **Timeout** safeguards: if the arbiter exceeds its time budget, the previous verdict persists for one tick, then the system defaults to BLOCK.

The optional Safety Mindset applies contextual caps before arbitration. Profiles such as `cautious_pedestrian`, `low_visibility`, and `worst_case_child` adjust maximum speed and minimum clearance. Uncertainty bias further reduces caps when lane confidence is low or perception is degraded.

Ambient mode gates motion when no explicit goal is set—movement occurs only when the arbiter returns ALLOW/AMEND, and throttle is limited to a gentle cruise. The riding companion narrates decisions every ≥2 s (e.g., “Fail-stop: pedestrian crossing ahead”).

- **normal** – Balanced thresholds. The arbiter AMENDs before BLOCKing when possible, and allows one tick of timeout grace.
- **strict** – Conservative behavior. Higher confidence required to ALLOW, faster BLOCK response on TTC, longer debounce, and zero timeout grace (defaults to BLOCK on overruns).

### Demo Scenarios

| Scenario | Expected Behavior |
| --- | --- |
| Obstacle 1.5 m ahead at cruising speed | Arbiter emits `BLOCK` within one tick, SafetyGate holds full brake until obstacle clears + debounce. Logged with reason `ttc_low` and incident entry. |
| Bike lane available on the right | Lane detector spots extra clearance, arbiter issues `AMEND` steering bias right with reduced throttle (`lane_bias_right`), SafetyGate publishes the amended command. |
| Forced arbiter timeout | Latency budget exceeded → previous verdict reused for ≤1 tick, then `BLOCK` with reason `timeout`. |
| Safety Mindset toggle | With mindset OFF, throttle follows Pilot target. Turning mindset ON lowers `caps_speed` and logs the active profile. |
| Ambient, uncertain lane | Low lane confidence with ambient ON → Arbiter `BLOCK`, scooter remains stopped until confidence recovers. |
| Companion narration | Messages such as “Fail-stop: pedestrian crossing ahead” appear in logs no more than once every 2 s. |

### Telemetry & Incident Logging

- `logs/telemetry.jsonl` records every tick with arbiter decision, reason tags, latency, proposed vs final command, caps, lane context, and navigation sub-goal state.
- `logs/incidents.jsonl` captures each AMEND/BLOCK event with a reference to 5 s pre/post clips (populate the clip fields if you archive video). Use these logs to audit fail-stop coverage and arbiter timing.
- Vehicle envelope metadata (lane width estimate, left/right clearance, required clearance) is emitted with each tick so you can audit why the arbiter held, amended, or released control.

## Extending the System

- **Additional sensors:** Feed their obstacle cues into `Navigator.plan` by augmenting the occupancy map.
- **Custom models:** Provide a different YOLO checkpoint via `--model` or tune `--lane-sensitivity` to match your optics and road surface.
- **Vehicle integration:** Map the normalized actuator outputs to your scooter's control API. For example, scale `steer` to handlebar servo angles and translate `throttle`/`brake` to PWM duty cycles.

## Safety Notice

This codebase is intended for research and prototyping. Always test in a controlled environment, keep a human operator ready to take over, and comply with local regulations for sidewalk and bike-lane operation.
