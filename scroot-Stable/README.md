# Minimal Autonomous Navigator

This project runs a lightweight YOLO model over an MP4 file (or any OpenCV-readable source), mimicking a live camera feed. It prints actuator outputs in the format `(steering, throttle, brake)` on every frame, paints those values on the overlay, and streams them to an Arduino if a serial port is provided. The first three seconds of operation emit `(0.000, 0.000, 0.000)` for calibration before autonomous driving begins.

## Requirements
- Python 3.8+
- FFmpeg and GL libraries for OpenCV video decoding (`ffmpeg`, `libgl1`, `libglib2.0-0` on Debian/Ubuntu/WSL)
- Optional NVIDIA GPU with CUDA if you want GPU inference (`--device cuda:0`)

## One-command setup

### Generic Linux / Jetson
```bash
(cd scroot && bash setup.sh)
```

### WSL
```bash
(cd scroot && bash setup_wsl.sh)
```

Both scripts install Ultralytics YOLO, OpenCV, NumPy, and PySerial; the WSL script also prepares a virtual environment and grabs the correct Torch build for CPU or CUDA.

## Running the navigator

After installing dependencies (and activating `.venv` if you used `setup_wsl.sh`), run:

```bash
python scroot/navigator.py --input /path/to/video.mp4 --display --serial-port /dev/ttyACM0
```

Key behaviors:
- Prints `(steering, throttle, brake)` each frame; during the initial 3 seconds, it prints `(0.000, 0.000, 0.000)` for calibration.
- Overlays detections plus actuator bars and the tuple at the top of the video window.
- Sends the same tuple over serial as `steering,throttle,brake\n` when `--serial-port` is provided.
- Press `q` to exit the overlay window.

Useful flags:
- `--device cpu` (default) or `--device cuda:0`
- `--model yolov8n.pt` (default lightweight YOLO)
- `--output overlay.mp4` to save the annotated video
- `--calibration 3.0` to adjust the zero-output duration
- `--baud-rate 115200` to match your Arduino settings
- Omit `--display` for headless runs (the actuator tuple still prints to stdout and over serial)

## Safety note
This code is for offline experimentation. Always validate outputs in a controlled setting before connecting to physical hardware.
