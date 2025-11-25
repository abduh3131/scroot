# Development Report

## Objective
Deliver a minimal autonomous navigator that treats an MP4 file as a live camera feed, overlays YOLO detections, outputs actuator tuples `(steering, throttle, brake)`, and streams those values to an Arduino. The system must idle at `(0, 0, 0)` for the first three seconds for calibration and remain easy to set up on Jetson, Linux, and WSL.

## Implementation Summary
- **Navigator core (`scroot/navigator.py`)**
  - Loads a lightweight YOLO model and runs inference on each video frame.
  - Computes steering, throttle, and brake values to bias away from obstacles while cruising.
  - Prints actuator outputs as `(steering, throttle, brake)`; during calibration, the tuple is `(0.000, 0.000, 0.000)`.
  - Renders the tuple plus bar gauges on the overlay and streams the same values to an Arduino over serial when configured.
  - Honors a configurable calibration window (default: 3 seconds) before enabling autonomous motion.

- **Setup scripts**
  - `scroot/setup.sh` installs the Python dependencies directly for generic Linux or Jetson environments.
  - `scroot/setup_wsl.sh` detects WSL, installs missing system packages when `apt` is available, prepares a virtual environment, and installs the correct Torch build for CPU or CUDA alongside YOLO, OpenCV, NumPy, and PySerial.

## Usage Notes
- Run `python scroot/navigator.py --input /path/to/video.mp4 --display` to see the overlay and live actuator tuple; add `--serial-port /dev/ttyACM0` to stream to an Arduino.
- Use `--output overlay.mp4` to save the annotated video; omit `--display` for headless processing.
- Adjust `--calibration` to change the duration of the zeroed actuator phase.

## Testing
- `python3 -m compileall scroot/navigator.py`
