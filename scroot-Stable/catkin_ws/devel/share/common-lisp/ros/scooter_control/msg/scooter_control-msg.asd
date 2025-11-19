
(cl:in-package :asdf)

(defsystem "scooter_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AICommand" :depends-on ("_package_AICommand"))
    (:file "_package_AICommand" :depends-on ("_package"))
    (:file "SensorHub" :depends-on ("_package_SensorHub"))
    (:file "_package_SensorHub" :depends-on ("_package"))
    (:file "YoloDetection" :depends-on ("_package_YoloDetection"))
    (:file "_package_YoloDetection" :depends-on ("_package"))
    (:file "YoloDetectionArray" :depends-on ("_package_YoloDetectionArray"))
    (:file "_package_YoloDetectionArray" :depends-on ("_package"))
  ))