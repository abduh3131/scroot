;; Auto-generated. Do not edit!


(when (boundp 'scooter_control::YoloDetectionArray)
  (if (not (find-package "SCOOTER_CONTROL"))
    (make-package "SCOOTER_CONTROL"))
  (shadow 'YoloDetectionArray (find-package "SCOOTER_CONTROL")))
(unless (find-package "SCOOTER_CONTROL::YOLODETECTIONARRAY")
  (make-package "SCOOTER_CONTROL::YOLODETECTIONARRAY"))

(in-package "ROS")
;;//! \htmlinclude YoloDetectionArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass scooter_control::YoloDetectionArray
  :super ros::object
  :slots (_header _detections _source_model ))

(defmethod scooter_control::YoloDetectionArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) ())
    ((:source_model __source_model) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   (setq _source_model (string __source_model))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:source_model
   (&optional __source_model)
   (if __source_model (setq _source_model __source_model)) _source_model)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; scooter_control/YoloDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ;; string _source_model
    4 (length _source_model)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; scooter_control/YoloDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;; string _source_model
       (write-long (length _source_model) s) (princ _source_model s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; scooter_control/YoloDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance scooter_control::YoloDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; string _source_model
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source_model (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get scooter_control::YoloDetectionArray :md5sum-) "478a9916436565eb349ccaef92e54ba5")
(setf (get scooter_control::YoloDetectionArray :datatype-) "scooter_control/YoloDetectionArray")
(setf (get scooter_control::YoloDetectionArray :definition-)
      "# An array of YOLO detections from the dual_yolo_node

std_msgs/Header header

# Array of individual detections
YoloDetection[] detections

# Source of the detection (e.g., \"yolov8n\", \"yolov8s\", \"fused\")
string source_model

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
MSG: scooter_control/YoloDetection
# Defines a single YOLO object detection

# Class label of the detected object
string class_label

# Confidence score of the detection
float32 confidence

# Bounding box coordinates (in pixel space)
# top-left corner
int32 x_min
int32 y_min
# bottom-right corner
int32 x_max
int32 y_max
float32 distance   # estimated distance to object in meters, -1 if unknown

")



(provide :scooter_control/YoloDetectionArray "478a9916436565eb349ccaef92e54ba5")


