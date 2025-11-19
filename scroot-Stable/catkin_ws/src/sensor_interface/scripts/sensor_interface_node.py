#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from sensor_interface.msg import SensorHub
import sys
import traceback

# Configuration
LIDAR_TOPIC = "/scan"
CAMERA_TOPIC = "/usb_cam/image_raw"
FUSED_TOPIC = "/sensor_hub/data"
NODE_NAME = "sensor_interface_node"
QUEUE_SIZE = 10
SLOP_SECONDS = 0.1  # Max time difference between messages
TOPIC_TIMEOUT = 10.0 # Seconds to wait for topics to appear
LIDAR_WATCHDOG_TIMEOUT = 3.0 # Seconds since last lidar message before warning

class SensorFusionNode:
    """
    A ROS node to synchronize and fuse sensor data from a LiDAR and a camera.
    It subscribes to LaserScan and Image topics, and publishes a custom
    SensorHub message containing the synchronized data.
    """
    def __init__(self):
        rospy.init_node(NODE_NAME, log_level=rospy.INFO)
        rospy.loginfo("Starting sensor fusion node.")

        self.last_lidar_time = rospy.Time.now()

        # Wait for topics to become available
        self.wait_for_topics()

        # Subscribers
        lidar_sub = message_filters.Subscriber(LIDAR_TOPIC, LaserScan)
        camera_sub = message_filters.Subscriber(CAMERA_TOPIC, Image)

        # Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, camera_sub],
            queue_size=QUEUE_SIZE,
            slop=SLOP_SECONDS
        )
        self.ts.registerCallback(self.synchronized_callback)

        # Publisher
        self.fused_pub = rospy.Publisher(FUSED_TOPIC, SensorHub, queue_size=QUEUE_SIZE)

        # Watchdog timer for LiDAR
        rospy.Timer(rospy.Duration(1), self.lidar_watchdog)

        rospy.loginfo("Node initialized. Waiting for synchronized messages...")

    def wait_for_topics(self):
        """
        Waits for the required sensor topics to be published.
        If topics are not available after a timeout, it logs a fatal error and exits.
        """
        rospy.loginfo(f"Waiting for topic {LIDAR_TOPIC}...")
        try:
            rospy.wait_for_message(LIDAR_TOPIC, LaserScan, timeout=TOPIC_TIMEOUT)
        except rospy.ROSException:
            rospy.logfatal(f"Timeout: Topic {LIDAR_TOPIC} not available. Is the LiDAR driver running?")
            sys.exit(1)

        rospy.loginfo(f"Waiting for topic {CAMERA_TOPIC}...")
        try:
            rospy.wait_for_message(CAMERA_TOPIC, Image, timeout=TOPIC_TIMEOUT)
        except rospy.ROSException:
            rospy.logfatal(f"Timeout: Topic {CAMERA_TOPIC} not available. Is the camera driver running?")
            sys.exit(1)
        
        rospy.loginfo("All required topics are now available.")

    def synchronized_callback(self, lidar_msg, camera_msg):
        """
        Callback for synchronized LiDAR and Camera messages.
        Fuses the data and publishes it on the /sensor_hub/data topic.
        """
        try:
            self.last_lidar_time = rospy.Time.now()
            
            # Log timestamp diagnostics
            lidar_time = lidar_msg.header.stamp
            camera_time = camera_msg.header.stamp
            time_diff = abs(lidar_time.to_sec() - camera_time.to_sec())
            rospy.logdebug(f"Fused messages. Lidar: {lidar_time.to_sec():.4f}, Cam: {camera_time.to_sec():.4f}, Diff: {time_diff:.4f}s")

            # Create the fused message
            fused_msg = SensorHub()

            # 1. Header
            fused_msg.header.stamp = rospy.Time.now()
            fused_msg.header.frame_id = "base_link" # Or another appropriate frame

            # 2. LiDAR data
            fused_msg.lidar_ranges = lidar_msg.ranges
            fused_msg.lidar_angle_min = lidar_msg.angle_min
            fused_msg.lidar_angle_max = lidar_msg.angle_max
            fused_msg.lidar_angle_increment = lidar_msg.angle_increment

            # 3. Camera data
            fused_msg.camera_frame = camera_msg

            # 4. Placeholder data
            fused_msg.imu_vector = Vector3(x=0.0, y=0.0, z=0.0) # TODO: Populate with real IMU data
            fused_msg.ultrasonic_distance = 0.0 # TODO: Populate with real ultrasonic data

            self.fused_pub.publish(fused_msg)

        except Exception as e:
            rospy.logerr("Exception in synchronized_callback:")
            rospy.logerr(traceback.format_exc())

    def lidar_watchdog(self, event):
        """
        Checks if LiDAR messages are still being received.
        If no message has been received for a configured duration, logs a warning.
        """
        time_since_last_lidar = (rospy.Time.now() - self.last_lidar_time).to_sec()
        if time_since_last_lidar > LIDAR_WATCHDOG_TIMEOUT:
            rospy.logwarn(
                f"No new LiDAR scan received for {time_since_last_lidar:.2f} seconds. "
                f"Is the LiDAR driver still running?"
            )

    def run(self):
        """
        Keeps the node running.
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        fusion_node = SensorFusionNode()
        fusion_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down sensor fusion node.")
    except Exception as e:
        rospy.logfatal("An unhandled exception occurred during node initialization.")
        rospy.logfatal(traceback.format_exc())

