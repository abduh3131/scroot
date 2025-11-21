#!/usr/bin/env python3
"""
Lightweight planner that listens to the AI-annotated image stream and YOLO
detections, produces actuator targets (throttle, steering, brake), publishes
them to the ROS graph, and logs them to a CSV file for downstream MCU
integration.
"""

import csv
import os
from datetime import datetime

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from scooter_control.msg import ActuatorCommand, YoloDetectionArray


class SimplePlannerNode:
    """Rule-based planner tuned for speed and simplicity."""

    def __init__(self):
        rospy.init_node("planner_node", anonymous=True)

        # --- Parameters ---
        self.max_steering = rospy.get_param("~max_steering", 0.35)  # radians
        self.base_throttle = rospy.get_param("~base_throttle", 0.35)
        self.caution_distance = rospy.get_param("~caution_distance", 2.5)
        self.stop_distance = rospy.get_param("~stop_distance", 1.0)
        self.csv_path = rospy.get_param("~log_path", "/tmp/actuator_plan.csv")

        # --- ROS Interfaces ---
        self.bridge = CvBridge()
        image_sub = message_filters.Subscriber(
            rospy.remap_name("/AI/annotated_image"), Image, queue_size=1
        )
        detections_sub = message_filters.Subscriber(
            rospy.remap_name("/AI/detections"), YoloDetectionArray, queue_size=5
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, detections_sub], queue_size=5, slop=0.15
        )
        self.sync.registerCallback(self.callback)

        self.actuator_pub = rospy.Publisher(
            "~/actuator_plan", ActuatorCommand, queue_size=10
        )

        # --- Logging ---
        self._ensure_log_ready()

        rospy.loginfo("Planner node ready: fast rule-based actuator output.")

    def _ensure_log_ready(self):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.log_file = open(self.csv_path, "a", newline="")
        self.csv_writer = csv.writer(self.log_file)
        if self.log_file.tell() == 0:
            self.csv_writer.writerow(
                [
                    "timestamp",
                    "throttle",
                    "steering",
                    "brake",
                    "nearest_label",
                    "nearest_distance_m",
                ]
            )
            self.log_file.flush()

    def _select_target(self, image, detections):
        """Return steering, throttle, brake and metadata based on detections."""
        height, width = image.shape[:2]
        center_x = width / 2.0

        # Choose the nearest detection with a valid distance; fall back to the widest box.
        chosen = None
        chosen_distance = float("inf")
        for det in detections.detections:
            distance = det.distance if det.distance > 0 else float("inf")
            if distance < chosen_distance:
                chosen = det
                chosen_distance = distance

        if chosen is None and detections.detections:
            # No distances available; pick the box closest to center
            chosen = min(
                detections.detections,
                key=lambda d: abs(((d.x_min + d.x_max) / 2.0) - center_x),
            )
            chosen_distance = -1.0

        if chosen is None:
            return 0.0, self.base_throttle, 0.0, "CLEAR", -1.0

        box_center_x = (chosen.x_min + chosen.x_max) / 2.0
        lateral_error = (box_center_x - center_x) / center_x
        steering = float(np.clip(lateral_error * self.max_steering, -self.max_steering, self.max_steering))

        throttle = self.base_throttle
        brake = 0.0

        if 0 < chosen_distance <= self.stop_distance:
            throttle = 0.0
            brake = 1.0
            source = f"BLOCK: {chosen.class_label}"
        elif 0 < chosen_distance <= self.caution_distance:
            scale = max(0.1, chosen_distance / self.caution_distance)
            throttle = self.base_throttle * scale
            brake = 1.0 - scale
            source = f"CAUTION: {chosen.class_label}"
        else:
            source = f"TRACK: {chosen.class_label}"

        return steering, throttle, brake, source, chosen_distance

    def callback(self, image_msg, detections_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as err:
            rospy.logwarn_throttle(5.0, "Planner cv_bridge error: %s", err)
            return

        steering, throttle, brake, source, distance = self._select_target(
            cv_image, detections_msg
        )

        cmd = ActuatorCommand()
        cmd.header = image_msg.header
        cmd.throttle = float(np.clip(throttle, 0.0, 1.0))
        cmd.steering = steering
        cmd.brake = float(np.clip(brake, 0.0, 1.0))
        cmd.source = source
        self.actuator_pub.publish(cmd)

        # CSV log for downstream MCU bridge.
        timestamp = datetime.utcfromtimestamp(image_msg.header.stamp.to_sec()).isoformat()
        self.csv_writer.writerow(
            [timestamp, cmd.throttle, cmd.steering, cmd.brake, source, distance]
        )
        self.log_file.flush()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = SimplePlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
