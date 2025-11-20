#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from scooter_control.msg import AICommand
# This is the custom message from the perception team.
# You may need to create this message definition in a separate package
# and add it as a dependency. For this example, we assume it exists.
# If not, you can replace it with sensor_msgs/LaserScan and adjust the code.
from scooter_control.msg import SensorHub

class CruiseControlNode:
    def __init__(self):
        rospy.init_node('cruise_control_node', anonymous=True)

        # --- Parameters ---
        self.target_speed = rospy.get_param('~cruise_control/target_speed', 1.5)
        self.slowdown_distance = rospy.get_param('~cruise_control/slowdown_distance', 3.0)
        self.stop_distance = rospy.get_param('~cruise_control/stop_distance', 0.5)
        self.lidar_frontal_angle = rospy.get_param('~cruise_control/lidar_frontal_angle', 20.0)

        # --- ROS Hooks ---
        # IMPORTANT: Replace 'your_sensor_package.msg' and 'SensorHub' with the correct ones
        self.sensor_sub = rospy.Subscriber("/sensor_hub/data", SensorHub, self.sensor_callback)
        self.command_pub = rospy.Publisher("/control/cruise_command", AICommand, queue_size=10)

        rospy.loginfo("Cruise control node initialized.")

    def sensor_callback(self, msg):
        """Callback for unified sensor data."""
        
        # Extract LiDAR data from the custom message
        ranges = np.array(msg.lidar_ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Calculate the indices for the frontal arc
        center_index = len(ranges) / 2
        arc_radius_indices = int(np.deg2rad(self.lidar_frontal_angle / 2.0) / angle_increment)
        
        start_index = max(0, center_index - arc_radius_indices)
        end_index = min(len(ranges) - 1, center_index + arc_radius_indices)
        
        frontal_arc = ranges[start_index:end_index]
        
        # Filter out invalid range values (e.g., 0 or inf)
        frontal_arc = frontal_arc[np.isfinite(frontal_arc) & (frontal_arc > 0)]

        if len(frontal_arc) == 0:
            frontal_distance = float('inf')
        else:
            frontal_distance = np.min(frontal_arc)

        # --- Speed Logic ---
        command = AICommand()
        command.steering_angle = 0.0  # Cruise control is not responsible for steering

        if frontal_distance < self.stop_distance:
            command.speed = 0.0
            command.command = "STOP"
        elif frontal_distance < self.slowdown_distance:
            # Linearly scale speed
            speed_ratio = (frontal_distance - self.stop_distance) / (self.slowdown_distance - self.stop_distance)
            command.speed = self.target_speed * speed_ratio
            command.command = "SLOWDOWN"
        else:
            command.speed = self.target_speed
            command.command = "CRUISE"
            
        self.command_pub.publish(command)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CruiseControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
