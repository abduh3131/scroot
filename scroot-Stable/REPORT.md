# Self-Contained YOLO Pilot Report

## Requirements
- Python 3.9+ on Linux (Ubuntu/Jetson) or Windows 11.
- USB/CSI camera supported by OpenCV.
- `pip install -r requirements.txt` (installs `ultralytics`, `opencv-python`, `numpy`).
- Optional ROS 1 publishing: `rospy` and `std_msgs` available in the environment.
- Jetson note: if `opencv-python` fails to install from wheels, use the preinstalled `python3-opencv` and run `pip install --no-deps ultralytics numpy`.

## Setup
1. From this folder, create and activate a virtual environment (recommended):
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
   ```
2. Install dependencies and let Ultralytics pull the default YOLO weights on first run:
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
3. Plug in the camera. For Jetson/CSI cams you can use a GStreamer string via `--pipeline` instead of `--camera`.

## Running the pilot
Run from this folder so imports resolve:
```bash
python -m autonomy.pilot --model yolov8n.pt --device auto --camera 0 --display --log actuator_log.jsonl
```
- `--device auto` lets Ultralytics pick CUDA on Jetson/desktop if available or fall back to CPU.
- Use `--pipeline "nvarguscamerasrc ! ... ! appsink"` for Jetson CSI cameras.
- Add `--ros-topic /actuators` to publish `[steer, throttle, brake]` as a `Float32MultiArray` when `rospy` is present.

## Expected behavior
- Maintains a cruise throttle (default 0.35) when the view is clear.
- Steers away from detected objects using their weighted horizontal offset.
- Reduces throttle as the largest detection fills more of the frame and applies full brake once it crosses the brake area threshold.
- Logs JSONL actuator commands when `--log` is provided and draws overlays when `--display` is set (press `q` to exit).

## Common errors and fixes
- **Unable to open camera**: verify the index (`--camera 0`/`1`), check `/dev/video*` permissions on Linux, or supply a valid GStreamer `--pipeline`.
- **YOLO weight download fails**: ensure internet access or pre-download a `.pt` file and pass its path to `--model`.
- **Torch/Ultralytics installs slowly on Jetson**: prefer a swapfile and keep `--device cpu` if CUDA is unavailable.
- **ROS import error**: remove `--ros-topic` or install `rospy`/`std_msgs` from your ROS distribution.
- **No GUI window in headless sessions**: omit `--display` and rely on the actuator log instead.
