# Navigator Program Report

## Purpose
This navigator consumes an MP4 video, performs lightweight YOLO detection, and produces actuator outputs (steering \[-1, 1\], throttle \[0, 1\], brake \[0, 1\]) suitable for a scooter or rover. It renders bounding boxes and actuator indicators on the video stream and can stream comma-separated actuator values to an Arduino.

## Architecture
- **`navigator.py`**
  - Loads a YOLO model (default `yolov8n.pt`) on CPU or GPU.
  - Runs an initial calibration window (default 3 seconds) outputting zeroed actuators.
  - Processes each frame to compute actuator values based on detection area and offset from the frame center.
  - Sends actuator triples over serial when a port is provided and logs values for monitoring.
  - Generates overlays with bounding boxes, actuator bars (throttle/brake), and a steering direction line.
  - Optional GUI display (`--display`) and MP4 recording (`--output`).

## Actuation Strategy
- **Cruise:** Base throttle of 0.35 keeps forward motion.
- **Steering:** Objects offset from center induce steering away, scaled by object area (larger obstacles steer more).
- **Throttle:** Reduced proportionally to detected obstacle area when above the caution threshold.
- **Brake:** Applies braking within the caution zone; full brake and stop when object area exceeds the stop threshold.
- **Safety:** Calibration keeps actuators at zero for the first few seconds to allow hardware alignment.

## Dependencies & Setup
- Install with the included `setup.sh` (generic Linux/Jetson) or `setup_wsl.sh` (WSL-aware). Both are Python 3.8+ compatible.
- `setup_wsl.sh` auto-installs system packages via `apt` when present, validates Python 3.8+, and selects the appropriate Torch build (CUDA if `nvidia-smi` is visible, otherwise CPU).
- Key Python packages: `ultralytics`, `opencv-python-headless`, `numpy`, `pyserial`, and the matching Torch build.
- Optional: ensure `ffmpeg` is available on the system for MP4 decoding if your OpenCV build requires it.

## Running
Example headless run that saves the overlay video:
```bash
python3 navigator.py --input input.mp4 --output overlay.mp4 --device cpu
```

Example with Arduino output and live window:
```bash
python3 navigator.py --input input.mp4 --serial-port /dev/ttyACM0 --display --device cuda:0
```
Press `q` to exit when the window is enabled.

## Output Format
Each processed frame emits:
```
<steering>,<throttle>,<brake>\n
```
The same values are displayed at the top of the overlay alongside actuator bars and steering indicator lines.
