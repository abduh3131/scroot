; Auto-generated. Do not edit!


(cl:in-package scooter_control-msg)


;//! \htmlinclude YoloDetectionArray.msg.html

(cl:defclass <YoloDetectionArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector scooter_control-msg:YoloDetection)
   :initform (cl:make-array 0 :element-type 'scooter_control-msg:YoloDetection :initial-element (cl:make-instance 'scooter_control-msg:YoloDetection)))
   (source_model
    :reader source_model
    :initarg :source_model
    :type cl:string
    :initform ""))
)

(cl:defclass YoloDetectionArray (<YoloDetectionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YoloDetectionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YoloDetectionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name scooter_control-msg:<YoloDetectionArray> is deprecated: use scooter_control-msg:YoloDetectionArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <YoloDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:header-val is deprecated.  Use scooter_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <YoloDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:detections-val is deprecated.  Use scooter_control-msg:detections instead.")
  (detections m))

(cl:ensure-generic-function 'source_model-val :lambda-list '(m))
(cl:defmethod source_model-val ((m <YoloDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:source_model-val is deprecated.  Use scooter_control-msg:source_model instead.")
  (source_model m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YoloDetectionArray>) ostream)
  "Serializes a message object of type '<YoloDetectionArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'source_model))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'source_model))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YoloDetectionArray>) istream)
  "Deserializes a message object of type '<YoloDetectionArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'scooter_control-msg:YoloDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source_model) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'source_model) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YoloDetectionArray>)))
  "Returns string type for a message object of type '<YoloDetectionArray>"
  "scooter_control/YoloDetectionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YoloDetectionArray)))
  "Returns string type for a message object of type 'YoloDetectionArray"
  "scooter_control/YoloDetectionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YoloDetectionArray>)))
  "Returns md5sum for a message object of type '<YoloDetectionArray>"
  "478a9916436565eb349ccaef92e54ba5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YoloDetectionArray)))
  "Returns md5sum for a message object of type 'YoloDetectionArray"
  "478a9916436565eb349ccaef92e54ba5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YoloDetectionArray>)))
  "Returns full string definition for message of type '<YoloDetectionArray>"
  (cl:format cl:nil "# An array of YOLO detections from the dual_yolo_node~%~%std_msgs/Header header~%~%# Array of individual detections~%YoloDetection[] detections~%~%# Source of the detection (e.g., \"yolov8n\", \"yolov8s\", \"fused\")~%string source_model~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: scooter_control/YoloDetection~%# Defines a single YOLO object detection~%~%# Class label of the detected object~%string class_label~%~%# Confidence score of the detection~%float32 confidence~%~%# Bounding box coordinates (in pixel space)~%# top-left corner~%int32 x_min~%int32 y_min~%# bottom-right corner~%int32 x_max~%int32 y_max~%float32 distance   # estimated distance to object in meters, -1 if unknown~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YoloDetectionArray)))
  "Returns full string definition for message of type 'YoloDetectionArray"
  (cl:format cl:nil "# An array of YOLO detections from the dual_yolo_node~%~%std_msgs/Header header~%~%# Array of individual detections~%YoloDetection[] detections~%~%# Source of the detection (e.g., \"yolov8n\", \"yolov8s\", \"fused\")~%string source_model~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: scooter_control/YoloDetection~%# Defines a single YOLO object detection~%~%# Class label of the detected object~%string class_label~%~%# Confidence score of the detection~%float32 confidence~%~%# Bounding box coordinates (in pixel space)~%# top-left corner~%int32 x_min~%int32 y_min~%# bottom-right corner~%int32 x_max~%int32 y_max~%float32 distance   # estimated distance to object in meters, -1 if unknown~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YoloDetectionArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:length (cl:slot-value msg 'source_model))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YoloDetectionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'YoloDetectionArray
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
    (cl:cons ':source_model (source_model msg))
))
