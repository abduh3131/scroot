# Development Report

## Overview
The scooter autonomy stack now runs on Python 3.8 and newer, spans Jetson Orin to desktop GPUs, and offers a friendlier desktop app. The GUI scrolls smoothly on every tab, applies Jetson-specific CUDA defaults when that hardware is detected, and stores state between sessions. A brand-new **Media Test** tab lets you upload any road clip or still photo, run it through the entire pilot, and save a video with steering, throttle, braking, lane cues, and narration painted on top &mdash; now to any folder you pick. Under the hood the openpilot-style lane detector now performs CLAHE/Canny preprocessing, caches its bird’s-eye warp, uses a fixed downward-tilted perspective that mirrors the proven setup, draws a translucent lane corridor plus a bird’s-eye inset, and feeds those offsets right back into the navigator so the pilot actively seeks the center of every detected lane. The YOLO stack can be forced to run on CPU when CUDA kernels are missing, and the advisor narration is a simple checkbox away.

## Program Features to Date
1. **Autonomy Pilot Core** – Camera ingestion, YOLO-based object detection, the new perspective-based lane detector, navigator, controller, safety mindset, telemetry logging, and riding companion narration all run inside `AutonomyPilot`.
2. **Safety Arbiter + Command Interface** – The deterministic arbiter in `autonomy/control/arbitration.py` issues ALLOW/AMEND/BLOCK verdicts, while the natural-language parser turns phrases like “drive 2 m forward” into structured goals.
3. **Jetson-Oriented Optimizations** – The GUI auto-detects Jetson Orin hardware, switches to CUDA-ready dependency stacks, lowers resolution/FPS, and automatically loads the tuned lane-detector/safety settings so inference stays smooth.
4. **Scrollable Setup & Launch Tabs** – Every panel in the GUI shares the same mouse-wheel handler, so you can breeze through large forms on small displays or touchpads.
5. **Lane-Aware Navigator** – Lane curvature/offset measurements are now injected back into the navigator’s steering bias so the pilot hugs available lanes or shoulders automatically, while lane-induced hazard penalties are capped to prevent needless braking when the overlay shifts.
6. **Media Test Harness** – Upload a `.mp4/.mov/.avi/.mkv` clip or `.png/.jpg/.bmp` photo, then click **Generate Overlay** to replay that media through the pilot and save a narrated overlay video anywhere you want (or keep the `logs/media_tests/` default).
7. **Toggles for Lightweight Runs** – A CPU/auto/CUDA acceleration dropdown keeps YOLO from tripping CUDA errors, and an “Enable Advisor” checkbox lets you silence narration entirely when you want the leanest control loop.
8. **Documentation Refresh** – README, GUI guide, and this report now describe the new toggles, the lane-aware planner, and the offline replay flow in approachable language.

## Media Testing Workflow
1. Open the **Media Test** tab after setting up your launch preferences (camera resolution, lane profile, mindset, persona).
2. Use **Browse** to pick any road video or still photo. Images are treated as a short clip so the overlay renderer still produces a video file.
3. Press **Generate Overlay**. The exporter reuses your launch settings, applies Jetson optimizations when needed, and writes the result to either your chosen path or `logs/media_tests/<name>_<timestamp>.mp4` when left blank.
4. Watch the preview panel update in real time. The log explains each step (loading media, running inference, writing frames) so you can diagnose codec or dependency issues instantly.

## Lane Detector: How & Why
The refreshed lane detector lives inside `autonomy/perception/lane_detection.py` and feeds both the navigator and arbiter. Each frame now goes through a six-stage pipeline that stays lightweight enough for Python 3.8 laptops and Jetson Orin boards:

1. **Contrast-limited adaptive histogram equalization (CLAHE)** keeps lane paint visible even in dim or overexposed captures before we mask for white/yellow hues.
2. **Gradient + edge cues** from Sobel derivatives and Canny edges detect faded markings that color filters alone would miss.
3. **Cached bird’s-eye warp** avoids recomputing homographies when the resolution stays constant, saving CPU/GPU cycles on embedded hardware.
4. **Fixed downward-tilted horizon + warp** uses a hand-tuned trapezoid (tilted further toward the roadway) so the overlay hugs the pavement on a wide range of scooter camera heights without drifting toward the sky.
5. **Sliding-window fits with temporal smoothing** recreate both rails and compute curvature, width, and lateral offset in meters.
6. **Projection + visualization** paints the lane corridor back on the live feed, fills it with a translucent band, and produces a mini bird’s-eye inset so operators can sanity-check the detector instantly.

Those metrics mirror the cues openpilot’s `selfdrive/controls/lib/lateral_planner.py` uses. The navigator blends the offsets back into the steering bias so the pilot recenters inside the detected lane, and the arbiter continues logging reason tags such as `lane_bias_right`, `lane_too_narrow`, or `unknown_lane` whenever AMEND/BLOCK decisions fire.

## References
- openpilot `selfdrive/controls/lib/lateral_planner.py` – inspiration for translating lane confidence into lateral bias and safety caps.
- Ultralytics YOLOv8 – perception backbone for obstacle detection.

## Testing Notes
- `python -m compileall scroot-Stable/scroot`

## Next Steps
- Extend the media harness with batch processing so large datasets can be processed overnight.
- Add lightweight unit tests around the media exporter to guard against codec regressions on Python 3.8.
- Integrate localization and mapping sources so goal phrases like “drive to the plaza” follow GPS breadcrumbs.
