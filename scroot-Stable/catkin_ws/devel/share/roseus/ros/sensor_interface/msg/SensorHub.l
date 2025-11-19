;; Auto-generated. Do not edit!


(when (boundp 'sensor_interface::SensorHub)
  (if (not (find-package "SENSOR_INTERFACE"))
    (make-package "SENSOR_INTERFACE"))
  (shadow 'SensorHub (find-package "SENSOR_INTERFACE")))
(unless (find-package "SENSOR_INTERFACE::SENSORHUB")
  (make-package "SENSOR_INTERFACE::SENSORHUB"))

(in-package "ROS")
;;//! \htmlinclude SensorHub.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sensor_interface::SensorHub
  :super ros::object
  :slots (_header _lidar_ranges _lidar_angle_min _lidar_angle_max _lidar_angle_increment _camera_frame _imu_vector _ultrasonic_distance ))

(defmethod sensor_interface::SensorHub
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lidar_ranges __lidar_ranges) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:lidar_angle_min __lidar_angle_min) 0.0)
    ((:lidar_angle_max __lidar_angle_max) 0.0)
    ((:lidar_angle_increment __lidar_angle_increment) 0.0)
    ((:camera_frame __camera_frame) (instance sensor_msgs::Image :init))
    ((:imu_vector __imu_vector) (instance geometry_msgs::Vector3 :init))
    ((:ultrasonic_distance __ultrasonic_distance) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lidar_ranges __lidar_ranges)
   (setq _lidar_angle_min (float __lidar_angle_min))
   (setq _lidar_angle_max (float __lidar_angle_max))
   (setq _lidar_angle_increment (float __lidar_angle_increment))
   (setq _camera_frame __camera_frame)
   (setq _imu_vector __imu_vector)
   (setq _ultrasonic_distance (float __ultrasonic_distance))
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
  (:lidar_angle_min
   (&optional __lidar_angle_min)
   (if __lidar_angle_min (setq _lidar_angle_min __lidar_angle_min)) _lidar_angle_min)
  (:lidar_angle_max
   (&optional __lidar_angle_max)
   (if __lidar_angle_max (setq _lidar_angle_max __lidar_angle_max)) _lidar_angle_max)
  (:lidar_angle_increment
   (&optional __lidar_angle_increment)
   (if __lidar_angle_increment (setq _lidar_angle_increment __lidar_angle_increment)) _lidar_angle_increment)
  (:camera_frame
   (&rest __camera_frame)
   (if (keywordp (car __camera_frame))
       (send* _camera_frame __camera_frame)
     (progn
       (if __camera_frame (setq _camera_frame (car __camera_frame)))
       _camera_frame)))
  (:imu_vector
   (&rest __imu_vector)
   (if (keywordp (car __imu_vector))
       (send* _imu_vector __imu_vector)
     (progn
       (if __imu_vector (setq _imu_vector (car __imu_vector)))
       _imu_vector)))
  (:ultrasonic_distance
   (&optional __ultrasonic_distance)
   (if __ultrasonic_distance (setq _ultrasonic_distance __ultrasonic_distance)) _ultrasonic_distance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _lidar_ranges
    (* 4    (length _lidar_ranges)) 4
    ;; float32 _lidar_angle_min
    4
    ;; float32 _lidar_angle_max
    4
    ;; float32 _lidar_angle_increment
    4
    ;; sensor_msgs/Image _camera_frame
    (send _camera_frame :serialization-length)
    ;; geometry_msgs/Vector3 _imu_vector
    (send _imu_vector :serialization-length)
    ;; float32 _ultrasonic_distance
    4
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
     ;; float32 _lidar_angle_min
       (sys::poke _lidar_angle_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lidar_angle_max
       (sys::poke _lidar_angle_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lidar_angle_increment
       (sys::poke _lidar_angle_increment (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; sensor_msgs/Image _camera_frame
       (send _camera_frame :serialize s)
     ;; geometry_msgs/Vector3 _imu_vector
       (send _imu_vector :serialize s)
     ;; float32 _ultrasonic_distance
       (sys::poke _ultrasonic_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _lidar_angle_min
     (setq _lidar_angle_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lidar_angle_max
     (setq _lidar_angle_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lidar_angle_increment
     (setq _lidar_angle_increment (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; sensor_msgs/Image _camera_frame
     (send _camera_frame :deserialize buf ptr-) (incf ptr- (send _camera_frame :serialization-length))
   ;; geometry_msgs/Vector3 _imu_vector
     (send _imu_vector :deserialize buf ptr-) (incf ptr- (send _imu_vector :serialization-length))
   ;; float32 _ultrasonic_distance
     (setq _ultrasonic_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get sensor_interface::SensorHub :md5sum-) "b0a1fb0ce2812b325de845d2687d9680")
(setf (get sensor_interface::SensorHub :datatype-) "sensor_interface/SensorHub")
(setf (get sensor_interface::SensorHub :definition-)
      "std_msgs/Header header
float32[] lidar_ranges
float32 lidar_angle_min
float32 lidar_angle_max
float32 lidar_angle_increment
sensor_msgs/Image camera_frame
geometry_msgs/Vector3 imu_vector
float32 ultrasonic_distance

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
")



(provide :sensor_interface/SensorHub "b0a1fb0ce2812b325de845d2687d9680")


