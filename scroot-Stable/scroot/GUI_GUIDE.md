# Scooter Autonomy GUI Reference

This guide walks through every control inside the desktop app so you know exactly what it configures and when to use it. Use it alongside the guided setup inside the application for a smooth first launch.

## 1. Setup Tab

The Setup tab prepares the autonomy stack for your hardware and vehicle.

### Hardware Summary & Rescan
- **Detected Hardware panel** &mdash; Shows the operating system, CPU/GPU details, and the compute tier that the bootstrapper selected. These values drive the recommended model intensity and dependency stack defaults.
- **Rescan Hardware** &mdash; Runs the detection again. Use this after moving the SD card to a different Jetson, switching from WSL to native Ubuntu, or upgrading components so the GUI can recommend fresh profiles.

### Vehicle Envelope Panel
These fields teach the safety arbiter how much space your vehicle occupies and how close it may drive to obstacles.

| Field | What it does | Tips |
| --- | --- | --- |
| **Description** | Human-readable name used in narration and logs. | e.g., `Delivery Scooter`, `Cargo Cart`.
| **Width / Length / Height (m)** | Physical dimensions in meters. The navigator and SafetyGate use them to reject paths that are too narrow or low. | Measure the widest/longest points, including accessories such as baskets or mirrors.
| **Side Margin (m)** | Extra clearance you want on *each* side. Added to the width when checking lanes and curbs. | Typical scooters use `0.20–0.35`. Increase for cargo trailers or shaky environments.
| **Calibration Distance (m)** | Distance from the camera to the vehicle when you capture a reference frame. Used to convert pixel width to meters. | Place the vehicle at a known distance (e.g., 2 m) straight in front of the camera.
| **Reference Width (px)** | Pixel width of the vehicle in that reference frame. Lets the arbiter reason about real-world spacing from video. | Take a calibration photo, measure with an image viewer, enter the pixel count.

Validation runs when you press **Save Setup**; the app warns if any value is missing or non-numeric.

### Model Intensity
- **Dropdown** picks the perception stack size (e.g., lightweight vs. performance). Higher tiers run larger YOLO models for better accuracy at the cost of compute. The recommendation is based on the detected hardware tier.
- **Description text** on the right explains the latency/accuracy trade-offs for the currently selected model.

### Dependency Stack
- **Dropdown** picks the pip requirement set tailored for your platform (Jetson, WSL, native Ubuntu variants). Includes CUDA-enabled wheels when supported.
- **Install Dependencies** runs the bootstrapper. It automatically creates or reuses the managed `.venv/` when system pip is locked (PEP 668) and installs the selected stack.
- **Save Setup** persists the entire setup tab (vehicle envelope, model profile, dependency choice) so the Launch tab uses these defaults next time.
- **Setup Log** at the bottom prints progress from scans and installs. Errors appear here with `[error]` prefixes.

## 2. Launch Tab

The Launch tab controls runtime options and lets you start/stop the autonomy loop.

### Camera Source
- **Camera Source** entry accepts either a numeric index (`0`, `1`, ...), a video file/RTSP URL, or the runtime SensorHub snapshot (default: `~/scroot/runtime_inputs/camera.jpg`).
- **How to find the index**:
  - On Ubuntu/WSL: run `ls /dev/video*` and try the lowest number that appears when the camera is plugged in.
  - With `v4l2-ctl --list-devices` you can see descriptive names mapped to `/dev/videoX` nodes.
  - On Jetson, USB cameras usually start at `0`; CSI cameras exposed through `nvarguscamerasrc` often require the `0` index through the GStreamer pipeline already configured in the launcher.

### Resolution & FPS
- **Resolution** fields set capture width and height in pixels.
- **FPS** sets the target frame rate for the capture loop.
- **Recommended defaults**: start with `1280 x 720` at `30 FPS`. Drop to `960 x 540` or `848 x 480` at `20–24 FPS` on Jetson Nano/WSL VMs if the Launch Log reports arbiter timeouts. Higher-tier GPUs can push `1920 x 1080` at `30 FPS`.

### Acceleration Mode
- **Acceleration dropdown** chooses how Ultralytics YOLO runs: `auto` (use CUDA when available), `cpu` (force a CUDA-free path that mirrors the screenshot error fix), `cuda` (insist on GPU execution), or `quadro_p520` (locks YOLO to `cuda:0` with conservative timing for that board). The setting feeds both the live pilot and the Media Test harness so you can keep everything CPU-bound on lightweight laptops or squeeze extra mileage out of legacy GPUs such as the Quadro P520.

### Visualization Toggle
- **Enable Visualization** displays OpenCV windows with overlays. Disable it when running headless or when the display overhead reduces FPS.

### Safety & Behavior Toggles
- **Lane Detector Profile** &mdash; Chooses how aggressively the openpilot-style detector trusts pixel evidence:
  - `precision` &mdash; Highest confidence threshold with aggressive smoothing; ideal for narrow bike lanes.
  - `balanced` &mdash; Default compromise tuned for mixed urban riding.
  - `aggressive` &mdash; Lower threshold with minimal smoothing so the pilot reacts quickly on twisty or poorly marked paths.
- **Safety Mindset** &mdash; Enables (`on`) or disables (`off`) the risk-based caps that clamp max speed and enforce minimum clearance. When enabled, mindset profiles (cautious pedestrian, low visibility, etc.) activate automatically based on context.
- **Ambient Mode** &mdash; When `on`, the scooter only creeps forward when the arbiter explicitly ALLOWS/AMENDS and no user goal is set. Turning it `off` lets the Pilot pursue goals aggressively even without arbiter encouragement.
- **Advisor toggle + Persona** &mdash; Use **Enable Advisor Narration** to turn the riding companion on/off. When enabled, the persona dropdown picks the narration tone:
  - `calm_safe` &mdash; Reassuring, concise safety callouts.
  - `smart_scout` &mdash; Analytical, includes more scene context.
  - `playful` &mdash; Light-hearted phrasing while still reporting arbiter verdicts. Messages remain rate-limited to ≤1 every 2 s.

### Lane Alignment & Warp
- **Auto horizon trim** keeps the warp trapezoid nudged toward the pavement using a bounded, smoothed estimate from the current frame. Turn it off if you want a purely manual calibration.
- **Manual tilt** shifts the horizon up/down by ±0.2 of the frame so you can compensate for unique camera heights or dashboards.
- **Four normalized points** represent the trapezoid used for the bird’s-eye transform. Enter numbers between `0.000` and `1.000` (0 = left/top, 1 = right/bottom). Adjust the “top” pair to change how far down the road the warp reaches, and the “bottom” pair to widen/narrow the corridor. Hit **Reset points** to return to the safe defaults.

### Launch Controls
- **Start Pilot** launches the autonomy stack using all settings from both tabs (camera, vehicle envelope, lane profile, mindset, etc.). The button disables itself while the system runs.
- **Stop Pilot** sends a stop signal to the background thread. Always use this before closing the application to ensure logs flush cleanly.
- **Launch Log** streams real-time telemetry, including actuator commands, arbiter verdicts (`ALLOW`, `AMEND`, `BLOCK`), reason tags (e.g., `ttc_low`, `vru_slow`), and narration lines.

### Telemetry + LiDAR Snapshot
- **Throttle / Brake / Steer** gauges mirror the live actuator command so you can confirm the pilot is smoothly modulating inputs.
- **Safety Arbiter** label shows the latest verdict plus reason tags if an AMEND/BLOCK occurred.
- **Lane Status** reports the detector confidence and lateral offset (positive = right of center, negative = left). The overlay remains cosmetic but this readout confirms the warp calibration is sensible.
- **LiDAR (closest | median | ultrasonic)** continuously summarizes the fused ranges coming from `~/scroot/runtime_inputs/lidar.npy` and the ultrasonic distance stored in `sensor_meta.json`. As soon as the ROS bridge writes the SensorHub data, the GUI prints a line such as `0.82 m min | 4.10 m med | US 0.55 m` both in the telemetry row and the Command Console so you can validate that the fused topic is feeding the pilot.

### Live Camera, Overlay, and Messaging
- **Live Feed panel** shows the current camera image with YOLO bounding boxes and, when the detector is confident, a translucent lane corridor plus a bird’s-eye inset. The banner color mirrors the arbiter verdict (green = ALLOW, amber = AMEND, red = BLOCK) and lists reason tags plus latency. When no lane is detected the overlay remains hidden and the video feed stays clean.
- **Command Messaging box** records every verdict change and directive in natural language. Use the entry field to send commands such as “drive 3 m forward”, “turn left”, or “stop” without leaving the GUI—the Command Interface parses them immediately.

## 3. Media Test Tab

- **Media file** lets you browse to any dashcam clip (`.mp4`, `.mov`, `.avi`, `.mkv`) or still photo (`.png`, `.jpg`, `.bmp`).
- **Overlay output** accepts an optional folder or filename. Leave it blank to reuse `logs/media_tests/`, or browse to any destination (the app will ensure the folder exists and append `.mp4` if needed).
- **Generate Overlay** replays that media through the same configuration you set on the Launch tab, so your offline run mirrors the live stack (lane profile, mindset, resolution, Jetson tuning, etc.).
- **Stop** immediately cancels the export, removes any partial overlay, and re-enables the start button so you can switch clips without waiting.
- **Preview panel** shows the most recent overlay frame with YOLO boxes, narration captions, and (when available) the lane overlay. It updates while the export runs.
- **Status text + log** explain exactly what the exporter is doing (loading media, applying the pilot, writing frames). When it finishes, the output `.mp4` path is printed so you can open or share it.

## 4. Best Practices & Troubleshooting

- **First run flow**: Run `python setup_scroot.py --skip-models`, activate `.venv/`, and execute `python -m pip install -r autonomy/requirements.txt` before launching `python scooter_app.py`. The bootstrapper still rescans hardware on each launch; rerun **Install Dependencies** if you switch profiles.
- **ROS sensor feed**: Bring the fused sensors online with `roslaunch sensor_interface sensor_interface_dynamic.launch`, mirror them with `./scroot/bridge/ros_sensor_bridge.py`, then start the pilot via `python3 scroot/main.py` so the GUI and CLI share the same runtime camera/LiDAR/metadata files.
- **Camera testing**: Use `python autonomy_launcher.py --visualize` after configuring the camera to confirm the feed before a full ride.
- **Arbiter feedback**: Watch the Launch Log for reason tags like `ttc_low`, `timeout`, or `vru_slow`. Frequent timeouts mean you should lower resolution/FPS or pick a lighter model profile.
- **Incident logs**: Anytime the arbiter AMENDs or BLOCKs, entries go to `logs/incidents.jsonl` with references to 5-second clips (if recorded) for quick review.

With these references, every toggle in the GUI should map cleanly to the safety, navigation, or hardware behavior you expect during a ride.
