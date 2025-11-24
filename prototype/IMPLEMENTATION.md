# Implementation Notes

This document summarizes how the prototype splits responsibilities across nodes
and how data flows from the Jetson to the Arduino.

## Node responsibilities

- `sensor_interface_node.py`
  - Subscribes to Jetson-published topics for annotated camera images, LiDAR
    scans, and YOLO detections.
  - Republishes them under `/scooter/*` topics so downstream nodes do not need to
    know the Jetson namespace.
- `navigator_node.py`
  - Consumes `/scooter/lidar_scan` and `/scooter/yolo_detections`.
  - Estimates the nearest obstacle distance and computes a steering bias from
    the horizontal centroid of YOLO detections.
  - Publishes a `geometry_msgs/Twist` with throttle (linear.x), brake
    (linear.y), and steering (angular.z). Parameters allow tuning cruise speed,
    stop distance, and publish rate.
- `actuator_bridge_node.py`
  - Subscribes to `/scooter/actuator_cmd`.
  - Scales normalized actuator values into PWM ranges and sends three 16-bit
    values over serial for an Arduino to consume.

## Sensor assumptions

- Annotated images are handled pass-through style; the navigator does not process
  them directly but they remain available for visualization or future image-based
  planning.
- LiDAR readings are reduced to the nearest valid distance for conservative
  braking.
- YOLO detections use `vision_msgs/Detection2DArray`; the node uses bounding box
  centers to compute a coarse lateral bias and keep the scooter away from crowded
  regions.

## Actuator mapping

- Steering: normalized `[-1, 1]` mapped to servo PWM range (default 1000–2000 µs).
- Throttle: normalized `[0, 1]` mapped to ESC PWM range (default 1000–2000 µs).
- Brake: normalized `[0, 1]` mapped to 0–255. MCUs that expect a different brake
  representation can adjust `_brake_pwm_*` params or modify the packet format.

## Extending the prototype

- Swap `Twist` for a dedicated custom message if you want explicit field names
  for steering, throttle, and brake.
- Replace the steering bias heuristic with a costmap or model-predictive control
  policy; the navigator already exposes rate/throttle/stop parameters for
  experimentation.
- Add synchronization in the sensor interface using `message_filters` if tighter
  timestamp coupling is required between LiDAR and YOLO detections.
- Mirror the PWM framing on the Arduino with `Serial.readBytes()` and drive
  `analogWrite`/`Servo` outputs accordingly.
