# Development Report

## Overview
The scooter autonomy stack now runs on Python 3.8 and newer, spans Jetson Orin to desktop GPUs, and offers a friendlier desktop app. The GUI scrolls smoothly on every tab, applies Jetson-specific CUDA defaults when that hardware is detected, and stores state between sessions. A brand-new **Media Test** tab lets you upload any road clip or still photo, run it through the entire pilot, and save a video with steering, throttle, braking, lane cues, and narration painted on top. Under the hood we removed the heavy BLIP/FLAN advisor and replaced it with an openpilot-style lane detector plus deterministic safety arbiter, making the whole stack lighter while improving explainability.

## Program Features to Date
1. **Autonomy Pilot Core** – Camera ingestion, YOLO-based object detection, the new perspective-based lane detector, navigator, controller, safety mindset, telemetry logging, and riding companion narration all run inside `AutonomyPilot`.
2. **Safety Arbiter + Command Interface** – The deterministic arbiter in `autonomy/control/arbitration.py` issues ALLOW/AMEND/BLOCK verdicts, while the natural-language parser turns phrases like “drive 2 m forward” into structured goals.
3. **Jetson-Oriented Optimizations** – The GUI auto-detects Jetson Orin hardware, switches to CUDA-ready dependency stacks, lowers resolution/FPS, and defaults to the precision lane profile so inference stays smooth.
4. **Scrollable Setup & Launch Tabs** – Every panel in the GUI shares the same mouse-wheel handler, so you can breeze through large forms on small displays or touchpads.
5. **Media Test Harness** – Upload a `.mp4/.mov/.avi/.mkv` clip or `.png/.jpg/.bmp` photo, then click **Generate Overlay** to replay that media through the pilot and save a narrated overlay video to `logs/media_tests/`.
6. **Documentation Refresh** – README, GUI guide, and this report now describe all tabs, the offline replay flow, hardware tuning, and the reasoning system in approachable language.

## Media Testing Workflow
1. Open the **Media Test** tab after setting up your launch preferences (camera resolution, lane profile, mindset, persona).
2. Use **Browse** to pick any road video or still photo. Images are treated as a short clip so the overlay renderer still produces a video file.
3. Press **Generate Overlay**. The exporter reuses your launch settings, applies Jetson optimizations when needed, and writes the result to `logs/media_tests/<name>_<timestamp>.mp4`.
4. Watch the preview panel update in real time. The log explains each step (loading media, running inference, writing frames) so you can diagnose codec or dependency issues instantly.

## Lane Detector: How & Why
The refreshed lane detector lives inside `autonomy/perception/lane_detection.py` and feeds both the navigator and arbiter. Each frame is blurred, converted to HLS, thresholded for white/yellow paint, and warped into a bird’s-eye view. Sliding windows gather lane pixels, second-order polynomials are fit for left/right boundaries, and the curves are projected back onto the live feed. That gives us lane width, curvature, and lateral offset in meters—exactly the cues openpilot’s `selfdrive/controls/lib/lateral_planner.py` uses, but implemented with pure OpenCV/Numpy so it still runs on Python 3.8 laptops and Jetson Orin boards. Those metrics tighten lateral clearances in the navigator and trigger arbiter reason tags such as `lane_bias_right`, `lane_too_narrow`, or `unknown_lane` whenever AMEND/BLOCK decisions fire.

## References
- openpilot `selfdrive/controls/lib/lateral_planner.py` – inspiration for translating lane confidence into lateral bias and safety caps.
- Ultralytics YOLOv8 – perception backbone for obstacle detection.

## Testing Notes
- `python -m compileall scroot-Stable/scroot`

## Next Steps
- Extend the media harness with batch processing so large datasets can be processed overnight.
- Add lightweight unit tests around the media exporter to guard against codec regressions on Python 3.8.
- Integrate localization and mapping sources so goal phrases like “drive to the plaza” follow GPS breadcrumbs.
