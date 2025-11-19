
(cl:in-package :asdf)

(defsystem "sensor_interface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AICommand" :depends-on ("_package_AICommand"))
    (:file "_package_AICommand" :depends-on ("_package"))
    (:file "SensorHub" :depends-on ("_package_SensorHub"))
    (:file "_package_SensorHub" :depends-on ("_package"))
  ))