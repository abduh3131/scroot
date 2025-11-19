; Auto-generated. Do not edit!


(cl:in-package sensor_interface-msg)


;//! \htmlinclude SensorHub.msg.html

(cl:defclass <SensorHub> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lidar_ranges
    :reader lidar_ranges
    :initarg :lidar_ranges
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (lidar_angle_min
    :reader lidar_angle_min
    :initarg :lidar_angle_min
    :type cl:float
    :initform 0.0)
   (lidar_angle_max
    :reader lidar_angle_max
    :initarg :lidar_angle_max
    :type cl:float
    :initform 0.0)
   (lidar_angle_increment
    :reader lidar_angle_increment
    :initarg :lidar_angle_increment
    :type cl:float
    :initform 0.0)
   (camera_frame
    :reader camera_frame
    :initarg :camera_frame
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (imu_vector
    :reader imu_vector
    :initarg :imu_vector
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (ultrasonic_distance
    :reader ultrasonic_distance
    :initarg :ultrasonic_distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass SensorHub (<SensorHub>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorHub>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorHub)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_interface-msg:<SensorHub> is deprecated: use sensor_interface-msg:SensorHub instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SensorHub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_interface-msg:header-val is deprecated.  Use sensor_interface-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lidar_ranges-val :lambda-list '(m))
(cl:defmethod lidar_ranges-val ((m <SensorHub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_interface-msg:lidar_ranges-val is deprecated.  Use sensor_interface-msg:lidar_ranges instead.")
  (lidar_ranges m))

(cl:ensure-generic-function 'lidar_angle_min-val :lambda-list '(m))
(cl:defmethod lidar_angle_min-val ((m <SensorHub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_interface-msg:lidar_angle_min-val is deprecated.  Use sensor_interface-msg:lidar_angle_min instead.")
  (lidar_angle_min m))

(cl:ensure-generic-function 'lidar_angle_max-val :lambda-list '(m))
(cl:defmethod lidar_angle_max-val ((m <SensorHub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_interface-msg:lidar_angle_max-val is deprecated.  Use sensor_interface-msg:lidar_angle_max instead.")
  (lidar_angle_max m))

(cl:ensure-generic-function 'lidar_angle_increment-val :lambda-list '(m))
(cl:defmethod lidar_angle_increment-val ((m <SensorHub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_interface-msg:lidar_angle_increment-val is deprecated.  Use sensor_interface-msg:lidar_angle_increment instead.")
  (lidar_angle_increment m))

(cl:ensure-generic-function 'camera_frame-val :lambda-list '(m))
(cl:defmethod camera_frame-val ((m <SensorHub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_interface-msg:camera_frame-val is deprecated.  Use sensor_interface-msg:camera_frame instead.")
  (camera_frame m))

(cl:ensure-generic-function 'imu_vector-val :lambda-list '(m))
(cl:defmethod imu_vector-val ((m <SensorHub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_interface-msg:imu_vector-val is deprecated.  Use sensor_interface-msg:imu_vector instead.")
  (imu_vector m))

(cl:ensure-generic-function 'ultrasonic_distance-val :lambda-list '(m))
(cl:defmethod ultrasonic_distance-val ((m <SensorHub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_interface-msg:ultrasonic_distance-val is deprecated.  Use sensor_interface-msg:ultrasonic_distance instead.")
  (ultrasonic_distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorHub>) ostream)
  "Serializes a message object of type '<SensorHub>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lidar_ranges))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'lidar_ranges))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lidar_angle_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lidar_angle_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lidar_angle_increment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'camera_frame) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu_vector) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ultrasonic_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorHub>) istream)
  "Deserializes a message object of type '<SensorHub>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lidar_ranges) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lidar_ranges)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lidar_angle_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lidar_angle_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lidar_angle_increment) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'camera_frame) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu_vector) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ultrasonic_distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorHub>)))
  "Returns string type for a message object of type '<SensorHub>"
  "sensor_interface/SensorHub")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorHub)))
  "Returns string type for a message object of type 'SensorHub"
  "sensor_interface/SensorHub")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorHub>)))
  "Returns md5sum for a message object of type '<SensorHub>"
  "b0a1fb0ce2812b325de845d2687d9680")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorHub)))
  "Returns md5sum for a message object of type 'SensorHub"
  "b0a1fb0ce2812b325de845d2687d9680")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorHub>)))
  "Returns full string definition for message of type '<SensorHub>"
  (cl:format cl:nil "std_msgs/Header header~%float32[] lidar_ranges~%float32 lidar_angle_min~%float32 lidar_angle_max~%float32 lidar_angle_increment~%sensor_msgs/Image camera_frame~%geometry_msgs/Vector3 imu_vector~%float32 ultrasonic_distance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorHub)))
  "Returns full string definition for message of type 'SensorHub"
  (cl:format cl:nil "std_msgs/Header header~%float32[] lidar_ranges~%float32 lidar_angle_min~%float32 lidar_angle_max~%float32 lidar_angle_increment~%sensor_msgs/Image camera_frame~%geometry_msgs/Vector3 imu_vector~%float32 ultrasonic_distance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorHub>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lidar_ranges) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'camera_frame))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu_vector))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorHub>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorHub
    (cl:cons ':header (header msg))
    (cl:cons ':lidar_ranges (lidar_ranges msg))
    (cl:cons ':lidar_angle_min (lidar_angle_min msg))
    (cl:cons ':lidar_angle_max (lidar_angle_max msg))
    (cl:cons ':lidar_angle_increment (lidar_angle_increment msg))
    (cl:cons ':camera_frame (camera_frame msg))
    (cl:cons ':imu_vector (imu_vector msg))
    (cl:cons ':ultrasonic_distance (ultrasonic_distance msg))
))
