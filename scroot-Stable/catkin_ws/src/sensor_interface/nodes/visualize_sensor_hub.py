#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header # Correct import for Header
from sensor_interface.msg import SensorHub
import sensor_msgs.point_cloud2 as pc2

class SensorHubVisualizer:
    """
    Subscribes to SensorHub messages and publishes RViz-compatible visualizations.
    """
    def __init__(self):
        """
        Initializes the node, publishers, and subscriber.
        """
        rospy.init_node('sensor_hub_visualizer', anonymous=True)

        self.bridge = CvBridge()

        # Publishers
        self.pc_pub = rospy.Publisher('/sensor_hub/pointcloud', PointCloud2, queue_size=10)
        self.image_pub = rospy.Publisher('/sensor_hub/fused_image', Image, queue_size=10)

        # Subscriber
        # Using a small queue_size and large buff_size for potentially large messages like images
        rospy.Subscriber('/sensor_hub/data', SensorHub, self.sensorhub_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo("SensorHub Visualizer node started. Subscribing to /sensor_hub/data.")

    def sensorhub_callback(self, msg):
        """
        Callback for SensorHub messages.
        Processes the message and triggers visualization publishers.
        """
        if not isinstance(msg, SensorHub):
            rospy.logerr("Received message is not of type SensorHub. Skipping.")
            return

        try:
            # Phase 1: LiDAR and Camera visualization
            self.publish_lidar_pointcloud(msg)
            self.publish_fused_image(msg)

            # Phase 2 hooks (to be implemented later)
            # self.overlay_imu(cv_image, msg.imu_vector)
            # self.overlay_ultrasonic(cv_image, msg.ultrasonic_distance)

        except Exception as e:
            rospy.logerr(f"Unhandled error in sensorhub_callback: {e}")

    def publish_lidar_pointcloud(self, hub_msg):
        """
        Converts LiDAR data from SensorHub to a PointCloud2 message and publishes it.
        """
        if not hub_msg.lidar_ranges:
            rospy.logwarn_throttle(5, "LiDAR ranges data is empty. Skipping PointCloud2 publication.")
            return

        points = []
        for i, r in enumerate(hub_msg.lidar_ranges):
            # Filter out invalid ranges (NaN, too close, or too far)
            # np.isnan(r) check is implicitly handled by the range check if r is a float
            if not (0.15 < r < 18.0):
                continue

            angle = hub_msg.lidar_angle_min + i * hub_msg.lidar_angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0 # LiDAR is treated as 2D planar on z=0
            points.append([x, y, z])

        if not points:
            rospy.logwarn_throttle(5, "No valid LiDAR points after filtering. Skipping PointCloud2 publication.")
            return

        # Create PointCloud2 message header
        header = Header() # Correct usage of Header
        header.stamp = hub_msg.header.stamp # Use the timestamp from the SensorHub message
        header.frame_id = "laser_link"

        try:
            pointcloud_msg = pc2.create_cloud_xyz32(header, points)
            self.pc_pub.publish(pointcloud_msg)
            rospy.loginfo_throttle(10, "Published PointCloud2 message to /sensor_hub/pointcloud.")
        except Exception as e:
            rospy.logerr(f"Error publishing PointCloud2 message: {e}")


    def publish_fused_image(self, hub_msg):
        """
        Creates an image with a LiDAR data overlay and publishes it.
        """
        if hub_msg.camera_frame.width == 0 or hub_msg.camera_frame.height == 0:
            rospy.logwarn_throttle(5, "Received empty camera_frame. Skipping fused image publication.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(hub_msg.camera_frame, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error converting camera_frame to OpenCV image: {e}")
            return

        # Add the LiDAR overlay to the image
        self._add_lidar_overlay(cv_image, hub_msg)

        try:
            fused_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            fused_image_msg.header = hub_msg.header # Use the timestamp and frame_id from the SensorHub message
            self.image_pub.publish(fused_image_msg)
            rospy.loginfo_throttle(10, "Published fused image to /sensor_hub/fused_image.")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error converting fused OpenCV image to ROS Image: {e}")

    def _add_lidar_overlay(self, image, hub_msg):
        """
        Draws a 1D 'radar' bar graph of LiDAR data at the bottom of the image.
        This is a qualitative overlay for intuitive visualization.
        """
        h, w, _ = image.shape
        overlay_height = 60
        start_y = h - overlay_height

        # Draw a semi-transparent background for better readability
        # Create a black rectangle and blend it with the image region
        overlay_bg = image[start_y:h, 0:w].copy()
        cv2.rectangle(overlay_bg, (0, 0), (w, overlay_height), (0, 0, 0), -1) # Black rectangle
        cv2.addWeighted(image[start_y:h, 0:w], 1.0, overlay_bg, 0.4, 0, image[start_y:h, 0:w])


        num_ranges = len(hub_msg.lidar_ranges)
        if num_ranges == 0:
            return

        # Downsample points to avoid overdrawing on the image, ensuring at least one point per pixel column
        # or a reasonable step if ranges are sparse
        step = max(1, num_ranges // (w // 2)) # Plot at most w/2 lines

        for i in range(0, num_ranges, step):
            r = hub_msg.lidar_ranges[i]

            if not (0.15 < r < 18.0):
                continue

            # Map angle to the horizontal pixel coordinate
            # Ensure num_ranges - 1 is not zero for single-point LiDARs, though unlikely
            if num_ranges > 1:
                angle_ratio = i / float(num_ranges - 1)
            else:
                angle_ratio = 0.5 # Center a single point

            x_pos = int(angle_ratio * (w - 1))

            # Map distance to color and bar height for visualization
            color = (0, 255, 0)  # Default Green for far objects
            bar_height = int(overlay_height * 0.2) # Default height for far objects

            if r < 2.0:
                color = (0, 0, 255)  # Red for close objects
                bar_height = int(overlay_height * 0.8 * (1 - r / 2.0)) # Scale height based on proximity
            elif r < 5.0:
                color = (0, 255, 255)  # Yellow for medium distance
                bar_height = int(overlay_height * 0.5 * (1 - (r - 2.0) / 3.0)) # Scale height

            bar_height = max(1, min(bar_height, overlay_height - 5)) # Ensure height is within bounds and at least 1 pixel

            cv2.line(image, (x_pos, h - 1), (x_pos, h - bar_height), color, 2)

        # Add text label for the overlay
        cv2.putText(image, "LiDAR", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

if __name__ == '__main__':
    try:
        SensorHubVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down SensorHub Visualizer node.")
    except Exception as e:
        rospy.logerr(f"Unhandled exception in main execution: {e}")