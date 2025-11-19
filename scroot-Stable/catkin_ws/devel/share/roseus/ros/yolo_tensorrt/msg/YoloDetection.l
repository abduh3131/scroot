;; Auto-generated. Do not edit!


(when (boundp 'yolo_tensorrt::YoloDetection)
  (if (not (find-package "YOLO_TENSORRT"))
    (make-package "YOLO_TENSORRT"))
  (shadow 'YoloDetection (find-package "YOLO_TENSORRT")))
(unless (find-package "YOLO_TENSORRT::YOLODETECTION")
  (make-package "YOLO_TENSORRT::YOLODETECTION"))

(in-package "ROS")
;;//! \htmlinclude YoloDetection.msg.html


(defclass yolo_tensorrt::YoloDetection
  :super ros::object
  :slots (_x1 _y1 _x2 _y2 _score _class_id _class_name ))

(defmethod yolo_tensorrt::YoloDetection
  (:init
   (&key
    ((:x1 __x1) 0.0)
    ((:y1 __y1) 0.0)
    ((:x2 __x2) 0.0)
    ((:y2 __y2) 0.0)
    ((:score __score) 0.0)
    ((:class_id __class_id) 0)
    ((:class_name __class_name) "")
    )
   (send-super :init)
   (setq _x1 (float __x1))
   (setq _y1 (float __y1))
   (setq _x2 (float __x2))
   (setq _y2 (float __y2))
   (setq _score (float __score))
   (setq _class_id (round __class_id))
   (setq _class_name (string __class_name))
   self)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:serialization-length
   ()
   (+
    ;; float32 _x1
    4
    ;; float32 _y1
    4
    ;; float32 _x2
    4
    ;; float32 _y2
    4
    ;; float32 _score
    4
    ;; int32 _class_id
    4
    ;; string _class_name
    4 (length _class_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x1
       (sys::poke _x1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y1
       (sys::poke _y1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x2
       (sys::poke _x2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y2
       (sys::poke _y2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _score
       (sys::poke _score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _class_id
       (write-long _class_id s)
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x1
     (setq _x1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y1
     (setq _y1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x2
     (setq _x2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y2
     (setq _y2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _score
     (setq _score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _class_id
     (setq _class_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get yolo_tensorrt::YoloDetection :md5sum-) "dbc1212069278b57502e5887e3923efe")
(setf (get yolo_tensorrt::YoloDetection :datatype-) "yolo_tensorrt/YoloDetection")
(setf (get yolo_tensorrt::YoloDetection :definition-)
      "float32 x1
float32 y1
float32 x2
float32 y2
float32 score
int32 class_id
string class_name
")



(provide :yolo_tensorrt/YoloDetection "dbc1212069278b57502e5887e3923efe")


