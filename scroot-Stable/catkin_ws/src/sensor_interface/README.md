# Sensor Interface Package for ROSMASTER X3

This package provides the core sensor fusion pipeline for a Yahboom ROSMASTER X3 robot equipped with an RPLidar S2-L and a USB camera, running on a Jetson Nano.

It is designed to be a robust and reliable source of synchronized sensor data for higher-level AI and navigation tasks.

## System Overview

### Hardware
*   **Robot:** Yahboom ROSMASTER X3
*   **Computer:** NVIDIA Jetson Nano
*   **LiDAR:** Slamtec RPLidar S2-L
*   **Camera:** Standard USB Webcam (/dev/video0)

### Software Stack Layout
*   `~/catkin_ws/src/sensor_interface`: This ROS package.
*   `~/catkin_ws/src/rplidar_ros`: ROS driver for the LiDAR.
*   `~/catkin_ws/src/usb_cam`: ROS driver for the camera.
*   `~/scroot/`: Directory for external AI/ML code which consumes the data from this package.
*   `~/Rosmaster-App/`: Yahboom-provided utilities and robot bringup code.

## Core Functionality

This package launches and manages three key ROS nodes:

1.  `rplidarNode`: The driver for the RPLidar S2-L.
2.  `usb_cam_node`: The driver for the USB camera.
3.  `sensor_interface_node.py`: A custom Python node that:
    *   Subscribes to `/scan` (from the LiDAR) and `/usb_cam/image_raw` (from the camera).
    *   Uses `message_filters.ApproximateTimeSynchronizer` to fuse the two streams.
    *   Publishes the combined data on the `/sensor_hub/data` topic using the custom `sensor_interface/SensorHub` message type.

## Build Instructions

To build this package, run `catkin_make` from the root of your workspace:

```bash
# From your home directory
cd ~/catkin_ws
catkin_make
```

After building, you must source the setup file in any new terminal where you intend to use ROS commands with this workspace.

```bash
source ~/catkin_ws/devel/setup.bash
```

## How to Run

### Single Command (Recommended)

The entire sensor stack can be launched with a single command. This will start the LiDAR driver, camera driver, and fusion node.

```bash
roslaunch sensor_interface sensor_interface_dynamic.launch
```

### Manual Mode (for debugging)

You can also run each component manually in separate terminals.

1.  **Terminal 1: ROS Master**
    ```bash
    roscore
    ```
2.  **Terminal 2: LiDAR**
    ```bash
    source ~/catkin_ws/devel/setup.bash
    rosrun rplidar_ros rplidarNode _serial_port:=/dev/rplidar _serial_baudrate:=1000000 _frame_id:=laser_link
    ```
3.  **Terminal 3: Camera**
    ```bash
    source ~/catkin_ws/devel/setup.bash
    rosrun usb_cam usb_cam_node _video_device:=/dev/webcam
    ```
4.  **Terminal 4: Fusion Node**
    ```bash
    source ~/catkin_ws/devel/setup.bash
    rosrun sensor_interface sensor_interface_node.py
    ```

## How to Verify

After launching the system, use these commands in a new, sourced terminal to verify its operation.

1.  **Check Nodes:**
    ```bash
    rosnode list
    ```
    *Expected Output:* `/rosout`, `/rplidarNode`, `/sensor_interface_node`, `/usb_cam` (and TF broadcasters).

2.  **Check Topics:**
    ```bash
    rostopic list
    ```
    *Expected Output:* Must include `/scan`, `/usb_cam/image_raw`, and `/sensor_hub/data`.

3.  **Check Fusion Rate:**
    ```bash
    rostopic hz /sensor_hub/data
    ```
    *Expected Output:* A steady rate of approximately 10 Hz.

4.  **Visualize in RViz (Optional):**
    ```bash
    rviz
    ```
    *   Set the "Global Options" -> "Fixed Frame" to `base_link`.
    *   Add a `LaserScan` display and set the topic to `/scan`.
    *   Add an `Image` display and set the topic to `/usb_cam/image_raw`.
    *   Add a `TF` display to see the frame relationships.

## AI Integration (`scroot`)

The AI code developed by Abdullah in `~/scroot` should subscribe to the `/sensor_hub/data` topic.

*   **Topic:** `/sensor_hub/data`
*   **Message Type:** `sensor_interface/SensorHub`

The AI application must be run from a terminal where the Catkin workspace has been sourced (`source ~/catkin_ws/devel/setup.bash`) to recognize the custom message type. See the example subscriber script in the project documentation for implementation details.
