#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import message_filters
from scooter_control.msg import AICommand, SensorHub, YoloDetectionArray # Import YoloDetectionArray

class ObjectAvoidanceNode:
    def __init__(self):
        rospy.init_node('object_avoidance_node', anonymous=True)

        # --- Parameters ---
        self.lidar_arc_angle = rospy.get_param('~object_avoidance/lidar_arc_angle', 180.0)
        self.smoothing_factor = rospy.get_param('~object_avoidance/smoothing_factor', 0.2)
        self.critical_obstacle_dist = rospy.get_param('~object_avoidance/critical_obstacle_distance', 1.5)
        self.ai_stop_distance = rospy.get_param("~ai_stop_distance", 1.5)
        
        # --- ROS Hooks ---
        # self.sensor_sub = rospy.Subscriber("/sensor_hub/data", SensorHub, self.sensor_callback) # Old subscriber
        
        # Synchronized subscribers for SensorHub and YOLO detections
        sensor_sub = message_filters.Subscriber("/sensor_hub/data", SensorHub)
        yolo_sub = message_filters.Subscriber("/AI/detections", YoloDetectionArray)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sensor_sub, yolo_sub],
            queue_size=10,
            slop=0.1 # Allow up to 0.1 seconds difference in message timestamps
        )
        self.ts.registerCallback(self.fused_data_callback) # New callback for synchronized data

        self.command_pub = rospy.Publisher("/control/object_avoidance", AICommand, queue_size=10)

        # --- Internal State ---
        self.last_steering_angle = 0.0
        rospy.loginfo("Object avoidance node initialized.")

    def fused_data_callback(self, sensor_msg, yolo_msg):
        """Callback for unified sensor data and YOLO detections."""
        self.latest_detections = yolo_msg # Store for potential future use or direct integration

        ranges = np.array(sensor_msg.lidar_ranges)
        angle_increment = sensor_msg.lidar_angle_increment

        # Replace invalid values (inf, nan) with a large number
        ranges[~np.isfinite(ranges)] = 100.0

        # --- Sector Calculation ---
        # Total number of LiDAR points
        num_points = len(ranges)
        center_index = num_points // 2

        # Indices for the total arc to consider (e.g., 180 degrees)
        total_arc_indices = int(np.deg2rad(self.lidar_arc_angle / 2.0) / angle_increment)
        arc_start = max(0, center_index - total_arc_indices)
        arc_end = min(num_points, center_index + total_arc_indices)

        # Divide the main arc into three smaller sectors: left, front, right
        sector_indices = (arc_end - arc_start) // 3
        
        front_start = arc_start + sector_indices
        front_end = front_start + sector_indices
        
        left_sector = ranges[arc_start:front_start]
        front_sector = ranges[front_start:front_end]
        right_sector = ranges[front_end:arc_end]

        # --- Decision Logic ---
        # Use average clearance, but you could also use max or other metrics
        avg_clearance_left = np.mean(left_sector) if len(left_sector) > 0 else 0
        avg_clearance_right = np.mean(right_sector) if len(right_sector) > 0 else 0
        min_clearance_front = np.min(front_sector) if len(front_sector) > 0 else float('inf')

        command = AICommand()
        command.speed = 0.0  # Not responsible for speed
        target_steering = 0.0
        
        # --- NEW: AI-based blocking using YOLO distances ---
        ai_blocked = False
        if yolo_msg is not None:
            for det in yolo_msg.detections:
                # focus on relevant obstacle classes
                if det.class_label in ["person", "bicycle", "car", "motorcycle",
                                       "bus", "truck"]:
                    if det.distance > 0.0 and det.distance < self.ai_stop_distance:
                        ai_blocked = True
                        break

        # Combine LiDAR and AI blocking
        is_blocked = (min_clearance_front < self.critical_obstacle_dist) or ai_blocked

        if is_blocked:
            # Obstacle is directly ahead, choose the side with more space
            if avg_clearance_left > avg_clearance_right:
                target_steering = 0.4  # Radians, steer left
                command.command = "AVOID_LEFT"
            else:
                target_steering = -0.4 # Radians, steer right
                command.command = "AVOID_RIGHT"
        else:
            # No immediate obstacle, command straight
            target_steering = 0.0
            command.command = "CLEAR"

        # --- Steering Smoothing ---
        # Use an exponential moving average to prevent jerky movements
        smoothed_steering = (self.smoothing_factor * target_steering) + \
                            ((1 - self.smoothing_factor) * self.last_steering_angle)
        self.last_steering_angle = smoothed_steering
        
        command.steering_angle = smoothed_steering
        self.command_pub.publish(command)

    def run(self):
        # Keep the node alive
        rospy.spin() # The callbacks handle the logic

if __name__ == '__main__':
    try:
        node = ObjectAvoidanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
