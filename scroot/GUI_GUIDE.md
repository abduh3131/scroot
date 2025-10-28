# Scooter Autonomy GUI Reference

This guide walks through every control inside the desktop app so you know exactly what it configures and when to use it. Use it alongside the guided setup inside the application for a smooth first launch.

## 1. Setup Tab

The Setup tab prepares the autonomy stack for your hardware and vehicle.

### Hardware Summary & Rescan
- **Detected Hardware panel** &mdash; Shows the operating system, CPU/GPU details, and the compute tier that the bootstrapper selected. These values drive the recommended model intensity and dependency stack defaults.
- **Rescan Hardware** &mdash; Runs the detection again. Use this after moving the SD card to a different Jetson, switching from WSL to native Ubuntu, or upgrading components so the GUI can recommend fresh profiles.

### Vehicle Envelope Panel
These fields teach the Advisor how much space your vehicle occupies and how close it may drive to obstacles.

| Field | What it does | Tips |
| --- | --- | --- |
| **Description** | Human-readable name used in narration and logs. | e.g., `Delivery Scooter`, `Cargo Cart`.
| **Width / Length / Height (m)** | Physical dimensions in meters. Advisor, navigator, and SafetyGate use them to reject paths that are too narrow or low. | Measure the widest/longest points, including accessories such as baskets or mirrors.
| **Side Margin (m)** | Extra clearance you want on *each* side. Added to the width when checking lanes and curbs. | Typical scooters use `0.20–0.35`. Increase for cargo trailers or shaky environments.
| **Calibration Distance (m)** | Distance from the camera to the vehicle when you capture a reference frame. Used to convert pixel width to meters. | Place the vehicle at a known distance (e.g., 2 m) straight in front of the camera.
| **Reference Width (px)** | Pixel width of the vehicle in that reference frame. Lets the Advisor reason about real-world spacing from video. | Take a calibration photo, measure with an image viewer, enter the pixel count.

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
- **Camera Source** entry accepts either a numeric index (`0`, `1`, ...) or a video file/RTSP URL.
- **How to find the index**:
  - On Ubuntu/WSL: run `ls /dev/video*` and try the lowest number that appears when the camera is plugged in.
  - With `v4l2-ctl --list-devices` you can see descriptive names mapped to `/dev/videoX` nodes.
  - On Jetson, USB cameras usually start at `0`; CSI cameras exposed through `nvarguscamerasrc` often require the `0` index through the GStreamer pipeline already configured in the launcher.

### Resolution & FPS
- **Resolution** fields set capture width and height in pixels.
- **FPS** sets the target frame rate for the capture loop.
- **Recommended defaults**: start with `1280 x 720` at `30 FPS`. Drop to `960 x 540` or `848 x 480` at `20–24 FPS` on Jetson Nano/WSL VMs if the Advisor reports latency spikes. Higher-tier GPUs can push `1920 x 1080` at `30 FPS`.

### Visualization Toggle
- **Enable Visualization** displays OpenCV windows with overlays. Disable it when running headless or when the display overhead reduces FPS.

### Safety & Behavior Toggles
- **Enable Advisor** &mdash; Turns the arbitration layer on/off. Keep it enabled for normal operation; disabling reverts to raw Pilot commands without ALLOW/AMEND/BLOCK safety decisions.
- **Advisor Mode** &mdash; Chooses how cautious the Advisor is:
  - `normal` &mdash; Balanced. Prefers AMEND (steer nudges, throttle trims) before BLOCK, allows one tick of grace if a computation overruns its budget, and requires moderate confidence to ALLOW.
  - `strict` &mdash; Conservative. Blocks immediately on timeouts or low confidence, raises TTC thresholds for fail-stops, extends the hold time before releasing a BLOCK, and expects higher sensor confidence for ALLOW decisions.
- **Advisor Model** &mdash; Picks the vision-language workload used by the Advisor overlay and narration:
  - `light` &mdash; BLIP-base + FLAN-T5-small. Lowest latency for CPU-only laptops or Jetson Nano.
  - `normal` &mdash; BLIP-large + FLAN-T5-base. Balanced reasoning on Jetson Xavier/NX or modern laptops.
  - `heavy` &mdash; BLIP-large + FLAN-T5-large. Richest commentary for desktop GPUs with generous VRAM.
- **Safety Mindset** &mdash; Enables (`on`) or disables (`off`) the risk-based caps that clamp max speed and enforce minimum clearance. When enabled, mindset profiles (cautious pedestrian, low visibility, etc.) activate automatically based on context.
- **Ambient Mode** &mdash; When `on`, the scooter only creeps forward when the Advisor explicitly ALLOWS/AMENDS and no user goal is set. Turning it `off` lets the Pilot pursue goals aggressively even without Advisor encouragement.
- **Companion Persona** &mdash; Picks the narration tone for status messages:
  - `calm_safe` &mdash; Reassuring, concise safety callouts.
  - `smart_scout` &mdash; Analytical, includes more scene context.
  - `playful` &mdash; Light-hearted phrasing while still reporting Advisor verdicts. Messages remain rate-limited to ≤1 every 2 s.

### Launch Controls
- **Start Pilot** launches the autonomy stack using all settings from both tabs (camera, vehicle envelope, advisor config, etc.). The button disables itself while the system runs.
- **Stop Pilot** sends a stop signal to the background thread. Always use this before closing the application to ensure logs flush cleanly.
- **Launch Log** streams real-time telemetry, including actuator commands, Advisor verdicts (`ALLOW`, `AMEND`, `BLOCK`), reason tags (e.g., `ttc_low`, `lane_bias_right`), and narration lines.

### Live Camera, Overlay, and Messaging
- **Live Feed panel** shows the current camera image with projected trajectory lines (center and envelope boundaries). The banner color mirrors the Advisor verdict (green = ALLOW, amber = AMEND, red = BLOCK) and lists reason tags plus latency.
- **Throttle/Brake gauges** update each tick to mirror actuator outputs. Green bars rise with throttle, red bars rise with braking, and the steer readout helps correlate servo motion with what you see on screen.
- **Gate tags & caps** appear inside the overlay text whenever the SafetyGate clamps throttle (`caps_speed`, `envelope_stop`, etc.) or when the Safety Mindset imposes a stricter limit.
- **Advisor Messaging box** records every verdict change and directive in natural language. Use the entry field to send commands such as “drive 3 m forward”, “turn left”, or “stop” without leaving the GUI—the Command Interface parses them immediately.

## 3. Best Practices & Troubleshooting

- **First run flow**: Run `python setup_scroot.py --skip-models`, activate `.venv/`, and execute `python -m pip install -r autonomy/requirements.txt` before launching `python scooter_app.py`. The bootstrapper still rescans hardware on each launch; rerun **Install Dependencies** if you switch profiles.
- **Camera testing**: Use `python autonomy_launcher.py --visualize` after configuring the camera to confirm the feed before a full ride.
- **Advisor feedback**: Watch the Launch Log for reason tags like `unknown_lane` or `timeout`. Frequent timeouts mean you should lower resolution/FPS or pick a lighter model profile.
- **Incident logs**: Anytime the Advisor AMENDs or BLOCKs, entries go to `logs/incidents.jsonl` with references to 5-second clips (if recorded) for quick review.

With these references, every toggle in the GUI should map cleanly to the safety, navigation, or hardware behavior you expect during a ride.
