;; Auto-generated. Do not edit!


(when (boundp 'scooter_control::AICommand)
  (if (not (find-package "SCOOTER_CONTROL"))
    (make-package "SCOOTER_CONTROL"))
  (shadow 'AICommand (find-package "SCOOTER_CONTROL")))
(unless (find-package "SCOOTER_CONTROL::AICOMMAND")
  (make-package "SCOOTER_CONTROL::AICOMMAND"))

(in-package "ROS")
;;//! \htmlinclude AICommand.msg.html


(defclass scooter_control::AICommand
  :super ros::object
  :slots (_speed _steering_angle _command ))

(defmethod scooter_control::AICommand
  (:init
   (&key
    ((:speed __speed) 0.0)
    ((:steering_angle __steering_angle) 0.0)
    ((:command __command) "")
    )
   (send-super :init)
   (setq _speed (float __speed))
   (setq _steering_angle (float __steering_angle))
   (setq _command (string __command))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ;; float32 _steering_angle
    4
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get scooter_control::AICommand :md5sum-) "b0232b7d0c8d7c3cb97587aa6b1c5ccf")
(setf (get scooter_control::AICommand :datatype-) "scooter_control/AICommand")
(setf (get scooter_control::AICommand :definition-)
      "# Defines the final command sent to the motor controller
# It includes a speed value and a steering angle.

# Speed in meters/second.
# Positive for forward, 0 for stop.
float32 speed

# Steering angle in radians.
# 0 is straight, positive is left, negative is right.
float32 steering_angle

# A command string for high-level instructions or status
# e.g., \"STOP\", \"CRUISE\", \"AVOID_LEFT\"
string command

")



(provide :scooter_control/AICommand "b0232b7d0c8d7c3cb97587aa6b1c5ccf")


