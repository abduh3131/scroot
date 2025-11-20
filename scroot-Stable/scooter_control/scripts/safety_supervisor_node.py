#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import message_filters
from scooter_control.msg import AICommand, SensorHub, YoloDetectionArray # Import YoloDetectionArray

class SafetySupervisorNode:
    def __init__(self):
        rospy.init_node('safety_supervisor_node', anonymous=True)

        # --- Parameters ---
        self.emergency_stop_dist = rospy.get_param('~safety_supervisor/emergency_stop_distance', 0.3)
        self.sensor_timeout_sec = rospy.get_param('~safety_supervisor/sensor_timeout', 0.5)
        self.sensor_timeout = rospy.Duration(self.sensor_timeout_sec)

        # --- ROS Hooks ---
        rospy.Subscriber("/control/proposed_command", AICommand, self.proposed_command_cb)
        # rospy.Subscriber("/sensor_hub/data", SensorHub, self.sensor_cb) # Old subscriber
        
        # Synchronized subscribers for SensorHub and YOLO detections
        sensor_sub = message_filters.Subscriber("/sensor_hub/data", SensorHub)
        yolo_sub = message_filters.Subscriber("/AI/detections", YoloDetectionArray)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sensor_sub, yolo_sub],
            queue_size=10,
            slop=0.1 # Allow up to 0.1 seconds difference in message timestamps
        )
        self.ts.registerCallback(self.fused_sensor_and_yolo_cb) # New callback for synchronized data
        
        self.final_command_pub = rospy.Publisher("/AI/AICommand", AICommand, queue_size=10)

        # --- Internal State ---
        self.latest_sensor_data = None
        self.last_sensor_timestamp = rospy.Time(0)
        self.last_proposed_cmd = AICommand() # Start with a safe, zero command
        self.latest_detections = None # Store latest YOLO detections

        rospy.loginfo("Safety supervisor node initialized.")
        rospy.Timer(rospy.Duration(1.0 / 20.0), self.check_and_publish)


    def fused_sensor_and_yolo_cb(self, sensor_msg, yolo_msg):
        """Stores the latest sensor data and YOLO detections."""
        self.latest_sensor_data = sensor_msg
        self.last_sensor_timestamp = sensor_msg.header.stamp
        self.latest_detections = yolo_msg # Store latest YOLO detections

    def check_and_publish(self, event):
        """
        Periodically checks safety conditions and publishes the final command.
        This runs on a timer to ensure a STOP is published even if other nodes fail.
        """
        final_command = self.last_proposed_cmd
        is_safe = True
        reason = ""

        # --- Safety Check 1: Sensor Timeout ---
        time_since_last_sensor = rospy.Time.now() - self.last_sensor_timestamp
        if self.last_sensor_timestamp != rospy.Time(0) and time_since_last_sensor > self.sensor_timeout:
            is_safe = False
            reason = "SAFETY: SENSOR_TIMEOUT"
            rospy.logwarn_throttle(1, reason)

        # --- Safety Check 2: Emergency LiDAR Stop ---
        if self.latest_sensor_data:
            ranges = np.array(self.latest_sensor_data.lidar_ranges)
            # Filter out invalid range values before finding the minimum
            valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0)]
            if len(valid_ranges) > 0:
                min_lidar_dist = np.min(valid_ranges)
                if min_lidar_dist < self.emergency_stop_dist:
                    is_safe = False
                    reason = "SAFETY: EMERGENCY_LIDAR_STOP"
                    rospy.logwarn_throttle(1, "%s - Distance: %.2f", reason, min_lidar_dist)

        # --- Safety Check 3: YOLO Detections ---
        if self.latest_detections and self.latest_detections.detections:
            # Placeholder: Implement logic to check for critical objects detected by YOLO
            # For example, if a "person" or "obstacle" is detected very close
            for det in self.latest_detections.detections:
                # This is a simplified example. Real logic would involve
                # projecting bounding boxes into 3D or checking their size/position.
                # Assuming a simple check on confidence and class for now.
                if det.class_label == "person" and det.confidence > 0.7:
                    # Example: if a person is detected with high confidence
                    # This needs more sophisticated logic to determine if it's "critical"
                    # For now, just a placeholder to show integration point
                    rospy.logwarn_throttle(1, "YOLO detected a critical object: %s with confidence %.2f", det.class_label, det.confidence)
                    # is_safe = False # Uncomment and refine this logic as needed
                    # reason = "SAFETY: YOLO_CRITICAL_OBJECT"
        
        # --- Override if Unsafe ---
        if not is_safe:
            final_command = AICommand() # Create a new zero-command
            final_command.speed = 0.0
            final_command.steering_angle = 0.0
            final_command.command = reason
        
        self.final_command_pub.publish(final_command)

if __name__ == '__main__':
    try:
        node = SafetySupervisorNode()
        rospy.spin() # The timer handles the publishing loop
    except rospy.ROSInterruptException:
        pass
