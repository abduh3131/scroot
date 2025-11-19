; Auto-generated. Do not edit!


(cl:in-package yolo_tensorrt-msg)


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
    :type (cl:vector yolo_tensorrt-msg:YoloDetection)
   :initform (cl:make-array 0 :element-type 'yolo_tensorrt-msg:YoloDetection :initial-element (cl:make-instance 'yolo_tensorrt-msg:YoloDetection))))
)

(cl:defclass YoloDetectionArray (<YoloDetectionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YoloDetectionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YoloDetectionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yolo_tensorrt-msg:<YoloDetectionArray> is deprecated: use yolo_tensorrt-msg:YoloDetectionArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <YoloDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo_tensorrt-msg:header-val is deprecated.  Use yolo_tensorrt-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <YoloDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo_tensorrt-msg:detections-val is deprecated.  Use yolo_tensorrt-msg:detections instead.")
  (detections m))
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
    (cl:setf (cl:aref vals i) (cl:make-instance 'yolo_tensorrt-msg:YoloDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YoloDetectionArray>)))
  "Returns string type for a message object of type '<YoloDetectionArray>"
  "yolo_tensorrt/YoloDetectionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YoloDetectionArray)))
  "Returns string type for a message object of type 'YoloDetectionArray"
  "yolo_tensorrt/YoloDetectionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YoloDetectionArray>)))
  "Returns md5sum for a message object of type '<YoloDetectionArray>"
  "4d47333e1297da1188ba7dfb186157b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YoloDetectionArray)))
  "Returns md5sum for a message object of type 'YoloDetectionArray"
  "4d47333e1297da1188ba7dfb186157b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YoloDetectionArray>)))
  "Returns full string definition for message of type '<YoloDetectionArray>"
  (cl:format cl:nil "std_msgs/Header header~%YoloDetection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: yolo_tensorrt/YoloDetection~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 score~%int32 class_id~%string class_name~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YoloDetectionArray)))
  "Returns full string definition for message of type 'YoloDetectionArray"
  (cl:format cl:nil "std_msgs/Header header~%YoloDetection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: yolo_tensorrt/YoloDetection~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 score~%int32 class_id~%string class_name~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YoloDetectionArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YoloDetectionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'YoloDetectionArray
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
