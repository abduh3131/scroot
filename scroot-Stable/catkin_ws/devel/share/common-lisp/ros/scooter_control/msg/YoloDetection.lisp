; Auto-generated. Do not edit!


(cl:in-package scooter_control-msg)


;//! \htmlinclude YoloDetection.msg.html

(cl:defclass <YoloDetection> (roslisp-msg-protocol:ros-message)
  ((class_label
    :reader class_label
    :initarg :class_label
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (x_min
    :reader x_min
    :initarg :x_min
    :type cl:integer
    :initform 0)
   (y_min
    :reader y_min
    :initarg :y_min
    :type cl:integer
    :initform 0)
   (x_max
    :reader x_max
    :initarg :x_max
    :type cl:integer
    :initform 0)
   (y_max
    :reader y_max
    :initarg :y_max
    :type cl:integer
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass YoloDetection (<YoloDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YoloDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YoloDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name scooter_control-msg:<YoloDetection> is deprecated: use scooter_control-msg:YoloDetection instead.")))

(cl:ensure-generic-function 'class_label-val :lambda-list '(m))
(cl:defmethod class_label-val ((m <YoloDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:class_label-val is deprecated.  Use scooter_control-msg:class_label instead.")
  (class_label m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <YoloDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:confidence-val is deprecated.  Use scooter_control-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'x_min-val :lambda-list '(m))
(cl:defmethod x_min-val ((m <YoloDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:x_min-val is deprecated.  Use scooter_control-msg:x_min instead.")
  (x_min m))

(cl:ensure-generic-function 'y_min-val :lambda-list '(m))
(cl:defmethod y_min-val ((m <YoloDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:y_min-val is deprecated.  Use scooter_control-msg:y_min instead.")
  (y_min m))

(cl:ensure-generic-function 'x_max-val :lambda-list '(m))
(cl:defmethod x_max-val ((m <YoloDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:x_max-val is deprecated.  Use scooter_control-msg:x_max instead.")
  (x_max m))

(cl:ensure-generic-function 'y_max-val :lambda-list '(m))
(cl:defmethod y_max-val ((m <YoloDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:y_max-val is deprecated.  Use scooter_control-msg:y_max instead.")
  (y_max m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <YoloDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:distance-val is deprecated.  Use scooter_control-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YoloDetection>) ostream)
  "Serializes a message object of type '<YoloDetection>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'class_label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'class_label))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'x_min)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_min)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x_max)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_max)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YoloDetection>) istream)
  "Deserializes a message object of type '<YoloDetection>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'class_label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'class_label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_min) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_min) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_max) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_max) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YoloDetection>)))
  "Returns string type for a message object of type '<YoloDetection>"
  "scooter_control/YoloDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YoloDetection)))
  "Returns string type for a message object of type 'YoloDetection"
  "scooter_control/YoloDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YoloDetection>)))
  "Returns md5sum for a message object of type '<YoloDetection>"
  "83c40f205e7d2dabf77dfc8fb221a703")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YoloDetection)))
  "Returns md5sum for a message object of type 'YoloDetection"
  "83c40f205e7d2dabf77dfc8fb221a703")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YoloDetection>)))
  "Returns full string definition for message of type '<YoloDetection>"
  (cl:format cl:nil "# Defines a single YOLO object detection~%~%# Class label of the detected object~%string class_label~%~%# Confidence score of the detection~%float32 confidence~%~%# Bounding box coordinates (in pixel space)~%# top-left corner~%int32 x_min~%int32 y_min~%# bottom-right corner~%int32 x_max~%int32 y_max~%float32 distance   # estimated distance to object in meters, -1 if unknown~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YoloDetection)))
  "Returns full string definition for message of type 'YoloDetection"
  (cl:format cl:nil "# Defines a single YOLO object detection~%~%# Class label of the detected object~%string class_label~%~%# Confidence score of the detection~%float32 confidence~%~%# Bounding box coordinates (in pixel space)~%# top-left corner~%int32 x_min~%int32 y_min~%# bottom-right corner~%int32 x_max~%int32 y_max~%float32 distance   # estimated distance to object in meters, -1 if unknown~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YoloDetection>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'class_label))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YoloDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'YoloDetection
    (cl:cons ':class_label (class_label msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':x_min (x_min msg))
    (cl:cons ':y_min (y_min msg))
    (cl:cons ':x_max (x_max msg))
    (cl:cons ':y_max (y_max msg))
    (cl:cons ':distance (distance msg))
))
