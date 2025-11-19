# scroot/ai

This folder exposes thin wrappers around the production autonomy stack so ROS
nodes and notebook experiments can import the SensorManager-style API without
reaching into implementation details.  Importing from `scroot.ai` gives you the
`AutonomyPilot`, file-backed `CameraSensor`, and the fused `LidarSensor` with its
`LidarSnapshot` metadata so downstream modules can process range, IMU, and
ultrasonic values that arrive from `/sensor_hub/data`.
