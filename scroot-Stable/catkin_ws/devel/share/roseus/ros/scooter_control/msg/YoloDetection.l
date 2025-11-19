;; Auto-generated. Do not edit!


(when (boundp 'scooter_control::YoloDetection)
  (if (not (find-package "SCOOTER_CONTROL"))
    (make-package "SCOOTER_CONTROL"))
  (shadow 'YoloDetection (find-package "SCOOTER_CONTROL")))
(unless (find-package "SCOOTER_CONTROL::YOLODETECTION")
  (make-package "SCOOTER_CONTROL::YOLODETECTION"))

(in-package "ROS")
;;//! \htmlinclude YoloDetection.msg.html


(defclass scooter_control::YoloDetection
  :super ros::object
  :slots (_class_label _confidence _x_min _y_min _x_max _y_max _distance ))

(defmethod scooter_control::YoloDetection
  (:init
   (&key
    ((:class_label __class_label) "")
    ((:confidence __confidence) 0.0)
    ((:x_min __x_min) 0)
    ((:y_min __y_min) 0)
    ((:x_max __x_max) 0)
    ((:y_max __y_max) 0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _class_label (string __class_label))
   (setq _confidence (float __confidence))
   (setq _x_min (round __x_min))
   (setq _y_min (round __y_min))
   (setq _x_max (round __x_max))
   (setq _y_max (round __y_max))
   (setq _distance (float __distance))
   self)
  (:class_label
   (&optional __class_label)
   (if __class_label (setq _class_label __class_label)) _class_label)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:x_min
   (&optional __x_min)
   (if __x_min (setq _x_min __x_min)) _x_min)
  (:y_min
   (&optional __y_min)
   (if __y_min (setq _y_min __y_min)) _y_min)
  (:x_max
   (&optional __x_max)
   (if __x_max (setq _x_max __x_max)) _x_max)
  (:y_max
   (&optional __y_max)
   (if __y_max (setq _y_max __y_max)) _y_max)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; string _class_label
    4 (length _class_label)
    ;; float32 _confidence
    4
    ;; int32 _x_min
    4
    ;; int32 _y_min
    4
    ;; int32 _x_max
    4
    ;; int32 _y_max
    4
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _class_label
       (write-long (length _class_label) s) (princ _class_label s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _x_min
       (write-long _x_min s)
     ;; int32 _y_min
       (write-long _y_min s)
     ;; int32 _x_max
       (write-long _x_max s)
     ;; int32 _y_max
       (write-long _y_max s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _class_label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _x_min
     (setq _x_min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_min
     (setq _y_min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x_max
     (setq _x_max (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_max
     (setq _y_max (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get scooter_control::YoloDetection :md5sum-) "83c40f205e7d2dabf77dfc8fb221a703")
(setf (get scooter_control::YoloDetection :datatype-) "scooter_control/YoloDetection")
(setf (get scooter_control::YoloDetection :definition-)
      "# Defines a single YOLO object detection

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



(provide :scooter_control/YoloDetection "83c40f205e7d2dabf77dfc8fb221a703")


