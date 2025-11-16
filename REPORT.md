# Development Report

## Overview
The scooter autonomy stack now runs on Python 3.8 and newer, spans Jetson Orin to desktop GPUs, and offers a friendlier desktop app. The GUI scrolls smoothly on every tab, applies Jetson-specific CUDA defaults when that hardware is detected, and stores state between sessions. A brand-new **Media Test** tab lets you upload any road clip or still photo, run it through the entire pilot, and save a video with steering, throttle, braking, lane cues, directives, and narration painted on top. These additions make it easier to debug the AI, explain its behavior in simple English, and verify that optimization work actually delivers “buttery smooth” performance on embedded boards.

## Program Features to Date
1. **Autonomy Pilot Core** – Camera ingestion, YOLO-based object detection, hybrid navigator, controller, advisor, safety mindset, telemetry logging, and riding companion narration all run inside `AutonomyPilot`.
2. **Safety Advisor + Command Interface** – BLIP + FLAN-T5 situational advisor issues ALLOW/AMEND/BLOCK verdicts, while the natural-language parser turns phrases like “drive 2 m forward” into structured goals.
3. **Jetson-Oriented Optimizations** – The GUI auto-detects Jetson Orin hardware, switches to CUDA-ready dependency stacks, lowers resolution/FPS, and leans on lightweight advisor profiles so inference stays smooth.
4. **Scrollable Setup & Launch Tabs** – Every panel in the GUI shares the same mouse-wheel handler, so you can breeze through large forms on small displays or touchpads.
5. **Media Test Harness** – Upload a `.mp4/.mov/.avi/.mkv` clip or `.png/.jpg/.bmp` photo, then click **Generate Overlay** to replay that media through the pilot and save a narrated overlay video to `logs/media_tests/`.
6. **Documentation Refresh** – README, GUI guide, and this report now describe all tabs, the offline replay flow, hardware tuning, and the reasoning system in approachable language.

## Media Testing Workflow
1. Open the **Media Test** tab after setting up your launch preferences (camera resolution, advisor mode, mindset, persona).
2. Use **Browse** to pick any road video or still photo. Images are treated as a short clip so the overlay renderer still produces a video file.
3. Press **Generate Overlay**. The exporter reuses your launch settings, applies Jetson optimizations when needed, and writes the result to `logs/media_tests/<name>_<timestamp>.mp4`.
4. Watch the preview panel update in real time. The log explains each step (loading media, running inference, writing frames) so you can diagnose codec or dependency issues instantly.

## Lane Detector: How & Why
The lane-clearance logic lives inside `autonomy/planning/navigator.py` and `autonomy/control/arbitration.py`. Each frame is divided into left/center/right buckets. The navigator counts detection area per bucket, estimates lane width from camera calibration (`VehicleEnvelope.estimate_lane_width`), and shrinks the left/right clearance whenever an obstacle intrudes. Those numbers flow into the advisor, which emits reason tags like `lane_bias_right`, `lane_too_narrow`, or `unknown_lane` when it AMENDs or BLOCKs a command. This design mirrors the openpilot project’s `selfdrive/controls/lib/lateral_planner.py`, where Comma’s stack fits lane-line polynomials and biases the vehicle away from clutter. We leaned on that reference to keep our own planner explainable: instead of copying the polynomial fit, we use bounding-box density plus calibration math to reach the same “give the rider more room” intuition, all while remaining lightweight enough for Python 3.8 laptops and Jetson boards.

## References
- openpilot `selfdrive/controls/lib/lateral_planner.py` – inspiration for translating lane confidence into lateral bias and safety caps.
- Ultralytics YOLOv8 – perception backbone for obstacle detection.
- Salesforce BLIP + Google FLAN-T5 – multimodal advisor stack.

## Testing Notes
- `python -m compileall scroot-Stable/scroot`

## Next Steps
- Extend the media harness with batch processing so large datasets can be processed overnight.
- Add lightweight unit tests around the media exporter to guard against codec regressions on Python 3.8.
- Integrate localization and mapping sources so goal phrases like “drive to the plaza” follow GPS breadcrumbs.
