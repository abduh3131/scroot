#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scooter_control.msg import AICommand

class LaneAssistNode:
    def __init__(self):
        rospy.init_node('lane_assist_node', anonymous=True)

        # --- Parameters ---
        hsv_lower = rospy.get_param('~lane_assist/hsv_lower_thresh', [0, 0, 180])
        hsv_upper = rospy.get_param('~lane_assist/hsv_upper_thresh', [255, 40, 255])
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.steering_gain = rospy.get_param('~lane_assist/steering_gain', 0.5)
        self.max_steering_angle = rospy.get_param('~lane_assist/max_steering_angle', 0.52)

        # --- ROS Hooks ---
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.command_pub = rospy.Publisher("/control/lane_assist", AICommand, queue_size=10)

        rospy.loginfo("Lane assist node initialized.")

    def image_callback(self, msg):
        """Callback for incoming image messages."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # 1. Pre-process image
        height, width, _ = cv_image.shape
        # Define a region of interest (ROI) - bottom half of the image
        roi_top = int(height / 2)
        roi = cv_image[roi_top:, :]
        
        # 2. Convert to HSV and mask
        hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.hsv_lower, self.hsv_upper)

        # 3. Find the center of the detected lane markings
        moments = cv2.moments(mask)
        
        lane_center_x = 0
        if moments["m00"] != 0:
            lane_center_x = int(moments["m10"] / moments["m00"])
        else:
            # No lane markings detected, don't publish a command
            return

        # 4. Calculate steering error
        # The center of the ROI's width
        roi_center_x = width / 2
        error = roi_center_x - lane_center_x
        
        # 5. Calculate steering command (proportional controller)
        # The error is in pixels, we scale it by a gain to get a steering angle
        # A negative error means the lane is to the right, so we need a negative (right) steer
        steering_adjustment = self.steering_gain * (error / width) 
        
        # Clamp the steering angle to the maximum allowed value
        clamped_steering = np.clip(steering_adjustment, -self.max_steering_angle, self.max_steering_angle)

        # 6. Publish command
        command = AICommand()
        command.speed = 0.0  # Lane assist is not responsible for speed
        command.steering_angle = clamped_steering
        command.command = "LANE_FOLLOW"
        self.command_pub.publish(command)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LaneAssistNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
