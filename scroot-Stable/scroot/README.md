# Minimal Autonomous Navigator

This repository contains a lightweight navigator that reads an MP4 video, runs a lightweight YOLO model, overlays detections and actuator indicators, and streams actuator values (steering, throttle, brake) to an Arduino. It defaults to CPU execution but can use a GPU when available.

## Features
- Calibration phase with zeroed actuators for the first 3 seconds.
- YOLO-based obstacle detection that modulates steering, throttle, and brake values (steering in `[-1, 1]`, throttle/brake in `[0, 1]`).
- Annotated video output with bounding boxes, actuator bars, and real-time numeric readouts.
- Serial output to an Arduino as comma-separated actuator triples.
- Optional GUI display (disabled by default) and optional annotated MP4 recording.

## Quickstart (Linux / Jetson / Generic)
1. Install dependencies:
   ```bash
   bash setup.sh
   ```

2. Run the navigator on an MP4 file:
   ```bash
   python3 navigator.py --input /path/to/video.mp4 --output overlay.mp4 --device cpu
   ```

3. To stream actuators to an Arduino (e.g., `/dev/ttyACM0`) and enable the on-screen window:
   ```bash
   python3 navigator.py --input /path/to/video.mp4 --serial-port /dev/ttyACM0 --display --device cuda:0
   ```

Press `q` to exit when the display window is enabled. When running headless, omit `--display` and use `--output` to save the overlay video.

## WSL Setup
A dedicated WSL-friendly installer (`setup_wsl.sh`) auto-detects CUDA availability and installs the matching Torch build, along with system packages such as `ffmpeg` and `python3-venv` when `apt` is present.

```bash
bash setup_wsl.sh
source .venv/bin/activate
python navigator.py --input /path/to/video.mp4 --output overlay.mp4
```

The script prefers Python 3.8 when available and verifies that your interpreter is at least Python 3.8. If `nvidia-smi` is present inside WSL, it installs CUDA-enabled Torch; otherwise it installs the CPU wheel.

## Actuator Policy
- Cruises forward with a base throttle of `0.35`.
- Reduces throttle and steers away from objects proportional to their size and position.
- Engages braking when large or close obstacles are detected; comes to a stop when the obstacle area exceeds the stop threshold.

## Arduino Output Format
Every processed frame emits a single line over serial:
```
<steering>,<throttle>,<brake>\n
```
Example: `-0.120,0.320,0.000`. During calibration, all values are `0.000`.

## Notes
- The default model is `yolov8n.pt` (downloaded automatically by Ultralytics on first use). You can point to a custom lightweight model with `--model`.
- Ensure FFmpeg/OpenCV can decode your MP4 files; install `ffmpeg` via your package manager if needed.
- Tested with Python 3.8; Jetson devices can run on CPU if CUDA is unavailable, and WSL users can rely on `setup_wsl.sh` to pick the right Torch wheel.
