# Prototype ROS Pipeline Setup

This guide walks through the end-to-end flow for the prototype ROS pipeline that
separates the planner/navigator from the Jetson sensor interface and publishes
actuator values for an Arduino-class MCU. All commands assume ROS is installed
and sourced (e.g., `source /opt/ros/noetic/setup.bash`) and that you are inside
the `scroot` workspace.

## Directory layout

- `prototype/sensor_interface_node.py` — Bridges annotated images, LiDAR scans,
  and YOLO detections from the Jetson namespace into a local `/scooter` namespace.
- `prototype/navigator_node.py` — Subscribes to the bridged topics and emits
  normalized actuator commands (`steer`, `throttle`, `brake`) as a `Twist`.
- `prototype/actuator_bridge_node.py` — Converts the `Twist` commands to PWM
  values and streams them to an MCU over serial.
- `prototype/IMPLEMENTATION.md` — Notes on the design decisions and message
  mapping.

## Dependencies

- ROS 1 with `rospy`, `sensor_msgs`, `geometry_msgs`, `vision_msgs`, and
  `message_filters` available.
- Python `pyserial` for the actuator bridge: `python -m pip install pyserial`.
- Access to the Jetson topics providing the annotated image, LiDAR scan, and
  YOLO detections.

## Launch order

1. Start `roscore` if it is not already running.
2. Launch the Jetson-side publishers that produce:
   - `/jetson/annotated_image` (`sensor_msgs/Image`)
   - `/jetson/lidar_scan` (`sensor_msgs/LaserScan`)
   - `/jetson/yolo_detections` (`vision_msgs/Detection2DArray`)
3. On the receiving machine (or directly on the Jetson), run the bridge:
   ```bash
   rosrun scroot sensor_interface_node.py _jetson_namespace:=jetson
   ```
   Adjust `_jetson_namespace` if the upstream topics use a different prefix.
4. Start the navigator so it subscribes to the bridged topics and emits actuator
   commands:
   ```bash
   rosrun scroot navigator_node.py _rate_hz:=10 _stop_distance:=1.0 _cruise_throttle:=0.25
   ```
5. Finally, launch the actuator bridge so commands are converted to PWM and sent
   to the MCU:
   ```bash
   rosrun scroot actuator_bridge_node.py _port:=/dev/ttyACM0 _baud:=115200 \
       _steer_pwm_min:=1000 _steer_pwm_max:=2000 _throttle_pwm_min:=1000 _throttle_pwm_max:=2000 \
       _brake_pwm_min:=0 _brake_pwm_max:=255
   ```

## Topic graph

- `/jetson/annotated_image` → `/scooter/annotated_image`
- `/jetson/lidar_scan` → `/scooter/lidar_scan`
- `/jetson/yolo_detections` → `/scooter/yolo_detections`
- `/scooter/lidar_scan` + `/scooter/yolo_detections` → `/scooter/actuator_cmd`
- `/scooter/actuator_cmd` → serial PWM stream to Arduino

## MCU framing

The actuator bridge packs three unsigned 16-bit fields (`steer`, `throttle`,
`brake`) in little-endian order using `struct.pack("<HHH", ...)`. Update the
Arduino firmware to parse three consecutive 16-bit integers and drive the PWM
pins accordingly.

## Testing

- Run the navigator without an MCU connected; watch `/scooter/actuator_cmd` in
  `rostopic echo` to confirm throttle ramps to the cruise value and brakes engage
  when objects enter within the stop distance.
- Loop back the serial port or use a USB logic analyzer to confirm PWM values are
  in the expected range as you tweak steering/throttle/brake parameters.
