# Unified perception message from the sensor fusion node

std_msgs/Header header

# LiDAR data
float32[] lidar_ranges
float32 angle_min
float32 angle_increment
float32 angle_max

# Ultrasonic sensor data
float32 ultrasonic_distance

# IMU data
geometry_msgs/Vector3 imu_vector

# Raw camera frame
sensor_msgs/Image camera_frame
