
(cl:in-package :asdf)

(defsystem "yolo_tensorrt-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "YoloDetection" :depends-on ("_package_YoloDetection"))
    (:file "_package_YoloDetection" :depends-on ("_package"))
    (:file "YoloDetectionArray" :depends-on ("_package_YoloDetectionArray"))
    (:file "_package_YoloDetectionArray" :depends-on ("_package"))
  ))