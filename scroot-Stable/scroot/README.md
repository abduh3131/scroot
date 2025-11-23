# Simple ROS-friendly YOLO pilot

This repository contains only the essentials for a YOLO-based pilot that:

- reads frames from a USB (or CSI) camera
- runs ultralytics YOLO for object detection (CPU or CUDA)
- computes simple steering / throttle / brake commands to keep cruising while avoiding large obstacles
- writes actuator commands to a JSONL log and optionally publishes them on a ROS topic

Everything else from previous iterations has been removed to keep the project small and focused.

## Requirements (full report)

### System
- Python 3.8+ (tested with CPython; ROS users typically run 3.8 on Noetic)
- A USB camera (or CSI camera on Jetson with a GStreamer pipeline string)
- Optional: ROS 1 (rospy, std_msgs) if you want actuator topics; not required for logging to disk

### Python packages
Install with pip (on Jetson, use the preinstalled pip under the JetPack Python):

```bash
python3 -m pip install -r requirements.txt
```

Packages included:
- `ultralytics` (brings YOLOv8 and depends on PyTorch; on Jetson install a compatible torch wheel first)
- `opencv-python` (frame capture and visualization)
- `numpy` (lightweight math helpers)

### Jetson Nano + ROS checklist
1. Update system tools and ROS (Noetic example):
   ```bash
   sudo apt update && sudo apt install -y python3-pip python3-opencv ros-noetic-ros-base ros-noetic-std-msgs
   ```
2. Install a Jetson-compatible PyTorch wheel (from NVIDIA’s wheel index) **before** installing ultralytics.
3. Install Python requirements:
   ```bash
   python3 -m pip install -r requirements.txt
   ```
4. Use a GStreamer pipeline for CSI cams, e.g.:
   ```
   gst-launch-1.0 nvarguscamerasrc ! video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! appsink
   ```
   Pass that pipeline to `--pipeline` on the pilot CLI.

### Windows 11 / Linux desktop checklist
- Ensure your USB camera shows up as `/dev/video*` (Linux) or a DirectShow device (Windows).
- Install Python 3.10+ and run `python -m pip install -r requirements.txt`.
- CUDA is optional; set `--device cpu` to force CPU-only.

## Running the pilot

A single CLI drives everything:

```bash
python -m autonomy.pilot \
  --model yolov8n.pt \
  --camera 0 \
  --width 640 --height 480 --fps 30 \
  --log actuator_log.jsonl \
  --ros-topic /actuator_cmd \
  --display
```

Flags:
- `--model`: YOLO checkpoint (downloaded automatically by ultralytics if missing).
- `--device`: `auto`, `cpu`, or `cuda`.
- `--pipeline`: Provide a GStreamer string instead of `--camera` when using Jetson CSI.
- `--log`: Path to a JSONL file for actuator commands (timestamped entries with steering/throttle/brake).
- `--ros-topic`: Publishes `[steering, throttle, brake]` as `std_msgs/Float32MultiArray` when `rospy` is present.
- `--display`: Shows an annotated preview window; press `q` to quit.

## What the navigator does
- Keeps a modest cruise throttle (default 0.35) when the view is clear.
- Steers away from the weighted centroid of detected objects.
- Reduces throttle as large objects fill more of the frame and applies full brake when a single detection covers a configurable fraction of the image (default 22%).

## Outputs
- JSONL actuator log: `{"ts": <unix>, "steering": x, "throttle": y, "brake": z}` per frame.
- Optional ROS topic with the same triplet for direct MCU/ROS integration.

## File layout
- `autonomy/actuator_output.py` – actuator command dataclass and publisher (file + optional ROS).
- `autonomy/camera.py` – USB/CSI camera reader with optional GStreamer pipeline.
- `autonomy/detector.py` – ultralytics YOLO wrapper returning structured detections.
- `autonomy/navigator.py` – minimal obstacle-aware steering/throttle/brake logic.
- `autonomy/pilot.py` – CLI entrypoint wiring camera → YOLO → navigator → outputs.
- `requirements.txt` – minimal Python dependencies.
