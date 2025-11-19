#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from scooter_control.msg import AICommand, YoloDetectionArray

class CommandArbitratorNode:
    def __init__(self):
        rospy.init_node('command_arbitrator_node', anonymous=True)

        # --- Parameters ---
        self.loop_rate = rospy.get_param('~loop_rate', 20) # Hz
        self.obstacle_timeout = rospy.Duration(rospy.get_param('~command_arbitrator/obstacle_timeout', 1.0))

        # --- ROS Hooks ---
        rospy.Subscriber("/control/cruise_command", AICommand, self.cruise_cb)
        rospy.Subscriber("/control/lane_assist", AICommand, self.lane_assist_cb)
        rospy.Subscriber("/control/object_avoidance", AICommand, self.object_avoidance_cb)
        rospy.Subscriber("/AI/detections", YoloDetectionArray, self.detections_cb)
        
        self.command_pub = rospy.Publisher("/control/proposed_command", AICommand, queue_size=10)

        # --- Internal State ---
        self.latest_cruise_cmd = AICommand()
        self.latest_lane_assist_cmd = AICommand()
        self.latest_object_avoidance_cmd = AICommand()
        self.yolo_obstacle_detected = False
        self.last_yolo_obstacle_time = rospy.Time(0)

        # Initialize with safe defaults
        self.latest_object_avoidance_cmd.command = "CLEAR"

        rospy.loginfo("Command arbitrator node initialized.")

    # --- Callbacks to store latest commands ---
    def cruise_cb(self, msg):
        self.latest_cruise_cmd = msg

    def lane_assist_cb(self, msg):
        self.latest_lane_assist_cmd = msg

    def object_avoidance_cb(self, msg):
        self.latest_object_avoidance_cmd = msg

    def detections_cb(self, msg):
        # Define which YOLO classes are considered critical obstacles
        critical_objects = ["person", "car", "truck", "bicycle"]
        
        self.yolo_obstacle_detected = False
        for detection in msg.detections:
            if detection.class_label in critical_objects:
                self.yolo_obstacle_detected = True
                self.last_yolo_obstacle_time = rospy.Time.now()
                break # Found one, no need to check further

    def arbitrate_and_publish(self):
        """Main arbitration logic loop."""
        proposed_command = AICommand()

        # --- Speed Arbitration ---
        # Speed is primarily determined by the cruise control node.
        proposed_command.speed = self.latest_cruise_cmd.speed
        
        # --- Steering Arbitration (Priority-based) ---
        
        # Check if YOLO has seen an obstacle recently
        yolo_override = self.yolo_obstacle_detected or \
                        (rospy.Time.now() - self.last_yolo_obstacle_time < self.obstacle_timeout)

        # Priority 1: LiDAR-based Object Avoidance
        if self.latest_object_avoidance_cmd.command != "CLEAR":
            proposed_command.steering_angle = self.latest_object_avoidance_cmd.steering_angle
            proposed_command.command = "ARBITRATOR: " + self.latest_object_avoidance_cmd.command
        
        # Priority 2: YOLO-based Object Detection (can trigger LiDAR avoidance logic)
        # This is an indirect priority. If YOLO sees something, we might trust the 
        # object avoidance steering command even if its own "CLEAR" status is stale.
        # A more advanced implementation could have YOLO publish its own avoidance command.
        # For now, we just use it as a flag to prefer the avoidance command.
        elif yolo_override:
             proposed_command.steering_angle = self.latest_object_avoidance_cmd.steering_angle
             proposed_command.command = "ARBITRATOR: YOLO_OBSTACLE"

        # Priority 3: Lane Assist
        else:
            proposed_command.steering_angle = self.latest_lane_assist_cmd.steering_angle
            proposed_command.command = "ARBITRATOR: LANE_FOLLOW"

        # --- Final Polish on Command ---
        # If speed is zero, the command should reflect that.
        if proposed_command.speed == 0:
            proposed_command.command = "ARBITRATOR: " + self.latest_cruise_cmd.command # e.g., STOP

        self.command_pub.publish(proposed_command)

    def run(self):
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            self.arbitrate_and_publish()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = CommandArbitratorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
