;; Auto-generated. Do not edit!


(when (boundp 'scooter_control::SensorHub)
  (if (not (find-package "SCOOTER_CONTROL"))
    (make-package "SCOOTER_CONTROL"))
  (shadow 'SensorHub (find-package "SCOOTER_CONTROL")))
(unless (find-package "SCOOTER_CONTROL::SENSORHUB")
  (make-package "SCOOTER_CONTROL::SENSORHUB"))

(in-package "ROS")
;;//! \htmlinclude SensorHub.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass scooter_control::SensorHub
  :super ros::object
  :slots (_header _lidar_ranges _angle_min _angle_increment _angle_max _ultrasonic_distance _imu_vector _camera_frame ))

(defmethod scooter_control::SensorHub
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lidar_ranges __lidar_ranges) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:angle_min __angle_min) 0.0)
    ((:angle_increment __angle_increment) 0.0)
    ((:angle_max __angle_max) 0.0)
    ((:ultrasonic_distance __ultrasonic_distance) 0.0)
    ((:imu_vector __imu_vector) (instance geometry_msgs::Vector3 :init))
    ((:camera_frame __camera_frame) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _lidar_ranges __lidar_ranges)
   (setq _angle_min (float __angle_min))
   (setq _angle_increment (float __angle_increment))
   (setq _angle_max (float __angle_max))
   (setq _ultrasonic_distance (float __ultrasonic_distance))
   (setq _imu_vector __imu_vector)
   (setq _camera_frame __camera_frame)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lidar_ranges
   (&optional __lidar_ranges)
   (if __lidar_ranges (setq _lidar_ranges __lidar_ranges)) _lidar_ranges)
  (:angle_min
   (&optional __angle_min)
   (if __angle_min (setq _angle_min __angle_min)) _angle_min)
  (:angle_increment
   (&optional __angle_increment)
   (if __angle_increment (setq _angle_increment __angle_increment)) _angle_increment)
  (:angle_max
   (&optional __angle_max)
   (if __angle_max (setq _angle_max __angle_max)) _angle_max)
  (:ultrasonic_distance
   (&optional __ultrasonic_distance)
   (if __ultrasonic_distance (setq _ultrasonic_distance __ultrasonic_distance)) _ultrasonic_distance)
  (:imu_vector
   (&rest __imu_vector)
   (if (keywordp (car __imu_vector))
       (send* _imu_vector __imu_vector)
     (progn
       (if __imu_vector (setq _imu_vector (car __imu_vector)))
       _imu_vector)))
  (:camera_frame
   (&rest __camera_frame)
   (if (keywordp (car __camera_frame))
       (send* _camera_frame __camera_frame)
     (progn
       (if __camera_frame (setq _camera_frame (car __camera_frame)))
       _camera_frame)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _lidar_ranges
    (* 4    (length _lidar_ranges)) 4
    ;; float32 _angle_min
    4
    ;; float32 _angle_increment
    4
    ;; float32 _angle_max
    4
    ;; float32 _ultrasonic_distance
    4
    ;; geometry_msgs/Vector3 _imu_vector
    (send _imu_vector :serialization-length)
    ;; sensor_msgs/Image _camera_frame
    (send _camera_frame :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[] _lidar_ranges
     (write-long (length _lidar_ranges) s)
     (dotimes (i (length _lidar_ranges))
       (sys::poke (elt _lidar_ranges i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _angle_min
       (sys::poke _angle_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_increment
       (sys::poke _angle_increment (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_max
       (sys::poke _angle_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ultrasonic_distance
       (sys::poke _ultrasonic_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Vector3 _imu_vector
       (send _imu_vector :serialize s)
     ;; sensor_msgs/Image _camera_frame
       (send _camera_frame :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[] _lidar_ranges
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lidar_ranges (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _lidar_ranges i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _angle_min
     (setq _angle_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_increment
     (setq _angle_increment (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_max
     (setq _angle_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ultrasonic_distance
     (setq _ultrasonic_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Vector3 _imu_vector
     (send _imu_vector :deserialize buf ptr-) (incf ptr- (send _imu_vector :serialization-length))
   ;; sensor_msgs/Image _camera_frame
     (send _camera_frame :deserialize buf ptr-) (incf ptr- (send _camera_frame :serialization-length))
   ;;
   self)
  )

(setf (get scooter_control::SensorHub :md5sum-) "8a7ff4c96a4d7dde852a8ffd7c0319b9")
(setf (get scooter_control::SensorHub :datatype-) "scooter_control/SensorHub")
(setf (get scooter_control::SensorHub :definition-)
      "# Unified perception message from the sensor fusion node

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

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

")



(provide :scooter_control/SensorHub "8a7ff4c96a4d7dde852a8ffd7c0319b9")


