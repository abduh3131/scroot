; Auto-generated. Do not edit!


(cl:in-package scooter_control-msg)


;//! \htmlinclude AICommand.msg.html

(cl:defclass <AICommand> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (steering_angle
    :reader steering_angle
    :initarg :steering_angle
    :type cl:float
    :initform 0.0)
   (command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass AICommand (<AICommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AICommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AICommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name scooter_control-msg:<AICommand> is deprecated: use scooter_control-msg:AICommand instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <AICommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:speed-val is deprecated.  Use scooter_control-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'steering_angle-val :lambda-list '(m))
(cl:defmethod steering_angle-val ((m <AICommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:steering_angle-val is deprecated.  Use scooter_control-msg:steering_angle instead.")
  (steering_angle m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <AICommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader scooter_control-msg:command-val is deprecated.  Use scooter_control-msg:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AICommand>) ostream)
  "Serializes a message object of type '<AICommand>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AICommand>) istream)
  "Deserializes a message object of type '<AICommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AICommand>)))
  "Returns string type for a message object of type '<AICommand>"
  "scooter_control/AICommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AICommand)))
  "Returns string type for a message object of type 'AICommand"
  "scooter_control/AICommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AICommand>)))
  "Returns md5sum for a message object of type '<AICommand>"
  "b0232b7d0c8d7c3cb97587aa6b1c5ccf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AICommand)))
  "Returns md5sum for a message object of type 'AICommand"
  "b0232b7d0c8d7c3cb97587aa6b1c5ccf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AICommand>)))
  "Returns full string definition for message of type '<AICommand>"
  (cl:format cl:nil "# Defines the final command sent to the motor controller~%# It includes a speed value and a steering angle.~%~%# Speed in meters/second.~%# Positive for forward, 0 for stop.~%float32 speed~%~%# Steering angle in radians.~%# 0 is straight, positive is left, negative is right.~%float32 steering_angle~%~%# A command string for high-level instructions or status~%# e.g., \"STOP\", \"CRUISE\", \"AVOID_LEFT\"~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AICommand)))
  "Returns full string definition for message of type 'AICommand"
  (cl:format cl:nil "# Defines the final command sent to the motor controller~%# It includes a speed value and a steering angle.~%~%# Speed in meters/second.~%# Positive for forward, 0 for stop.~%float32 speed~%~%# Steering angle in radians.~%# 0 is straight, positive is left, negative is right.~%float32 steering_angle~%~%# A command string for high-level instructions or status~%# e.g., \"STOP\", \"CRUISE\", \"AVOID_LEFT\"~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AICommand>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AICommand>))
  "Converts a ROS message object to a list"
  (cl:list 'AICommand
    (cl:cons ':speed (speed msg))
    (cl:cons ':steering_angle (steering_angle msg))
    (cl:cons ':command (command msg))
))
