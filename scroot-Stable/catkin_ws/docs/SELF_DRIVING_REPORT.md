# Self-Driving Stack Overview (Jetson Orin, Python 3.8)

## What Happens (simple terms)
- **Sensor interface** reads the camera and LiDAR, synchronizes them, and publishes `/sensor_hub/data` so any node can subscribe.
- **Planner** (`sensor_fusion_planner_node.py`) subscribes to `/sensor_hub/data`, runs a lightweight YOLO model on the camera frame, measures free space from LiDAR, and decides throttle, braking, and steering.
- **Controller** (`controller_node.py`) smooths the planner output, writes live actuator numbers to `~/controller_command.txt`, and republishes them on `/controller/actuator_values` for other nodes or hardware drivers.
- **GUI** (`live_overlay_gui.py`) shows the fused video feed with bounding boxes plus a tiny HUD with throttle/brake/steering values so you can see what the stack is doing in real time.

## Copy/Paste Setup for a fresh Jetson Orin
```bash
# 1) Base ROS + build tools (Noetic works with Python 3.8)
sudo apt update
sudo apt install -y build-essential python3-pip python3-opencv python3-rosdep python3-catkin-tools ros-noetic-desktop-full

# 2) Initialize rosdep for dependency resolution
sudo rosdep init || true
rosdep update

# 3) Clone the repo and build
mkdir -p ~/scroot && cd ~/scroot
# (copy this workspace folder here, or git clone if remote)
cd ~/scroot/scroot-Stable/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash

# 4) Install lightweight YOLO runtime (uses CUDA/TensorRT if available)
pip3 install --upgrade pip
pip3 install ultralytics
```

## How to Run (per terminal)
```bash
# Terminal A: core services
source ~/scroot/scroot-Stable/catkin_ws/devel/setup.bash
roscore

# Terminal B: sensor hub (camera + LiDAR)
source ~/scroot/scroot-Stable/catkin_ws/devel/setup.bash
rosrun sensor_interface sensor_interface_node.py _camera_topic:=/usb_cam/image_raw _lidar_topic:=/scan

# Terminal C: planner (YOLO + LiDAR fusion)
source ~/scroot/scroot-Stable/catkin_ws/devel/setup.bash
rosrun scooter_control sensor_fusion_planner_node.py _yolo_weights:=~/models/yolov8n.pt

# Terminal D: controller stream
source ~/scroot/scroot-Stable/catkin_ws/devel/setup.bash
rosrun scooter_control controller_node.py _command_log_path:=~/controller_command.txt

# Terminal E: live GUI overlay
source ~/scroot/scroot-Stable/catkin_ws/devel/setup.bash
rosrun scooter_control live_overlay_gui.py
```

### Notes
- Swap `_camera_topic` or `_lidar_topic` params if your topics differ.
- The planner publishes `/controller/command`; the controller repackages it for actuators and the GUI.
- Edit `~base_speed`, `~stop_distance`, and `~max_steering` on the planner to make it more/less aggressive.
