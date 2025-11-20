#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np # Added import

# Import the custom message
from sensor_interface.msg import SensorHub

class SensorFusionNode:
    def __init__(self):
        rospy.init_node('sensor_interface_node', anonymous=True)
        rospy.loginfo("Initializing Sensor Interface Node...")

        # --- Parameters from launch file ---
        self.lidar_topic = rospy.remap_name("lidar_input")
        self.camera_topic = rospy.remap_name("camera_input")
        self.output_topic = "/sensor_hub/data"
        self.fused_image_topic = "/sensor_hub/fused_image"
        
        # --- Publisher ---
        self.fused_pub = rospy.Publisher(self.output_topic, SensorHub, queue_size=10)
        self.fused_image_pub = rospy.Publisher(self.fused_image_topic, Image, queue_size=1)
        self.bridge = CvBridge()

        # --- Subscribers and Synchronizer ---
        rospy.loginfo(f"Waiting for Lidar topic: {self.lidar_topic}")
        rospy.loginfo(f"Waiting for Camera topic: {self.camera_topic}")
        
        try:
            # Wait for one message to ensure topics are active
            rospy.wait_for_message(self.lidar_topic, LaserScan, timeout=20.0)
            rospy.wait_for_message(self.camera_topic, Image, timeout=20.0)
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout waiting for topics. Please check that sensor nodes are running and publishing. Error: {e}")
            rospy.signal_shutdown("Required topics not available.")
            return

        rospy.loginfo("Topics detected. Setting up message filter synchronizer.")

        lidar_sub = message_filters.Subscriber(self.lidar_topic, LaserScan)
        camera_sub = message_filters.Subscriber(self.camera_topic, Image)

        # Use ApproximateTimeSynchronizer for robustness against differing frame rates
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, camera_sub],
            queue_size=10,
            slop=11.0  # Allow up to 11.0 seconds difference in message timestamps
        )
        self.ts.registerCallback(self.sensor_callback)

        rospy.loginfo("Sensor Interface Node initialized successfully.")

    def sensor_callback(self, lidar_msg, camera_msg):
        """
        Callback to fuse sensor data and publish.
        """
        # Log the timestamps of the incoming messages
        lidar_time = lidar_msg.header.stamp.to_sec()
        camera_time = camera_msg.header.stamp.to_sec()
        time_diff = abs(lidar_time - camera_time)
        rospy.loginfo(f"Lidar timestamp: {lidar_time:.4f}, Camera timestamp: {camera_time:.4f}, Difference: {time_diff:.4f}s")

        fused_msg = SensorHub()

        # 1. Populate Header
        fused_msg.header.stamp = rospy.Time.now()
        fused_msg.header.frame_id = "base_link" # A common frame for the robot base

        # 2. Populate Lidar Data from LaserScan message
        fused_msg.lidar_ranges = lidar_msg.ranges
        fused_msg.lidar_angle_min = lidar_msg.angle_min
        fused_msg.lidar_angle_max = lidar_msg.angle_max
        fused_msg.lidar_angle_increment = lidar_msg.angle_increment

        # 3. Populate Camera Data from Image message
        fused_msg.camera_frame = camera_msg

        # 4. Populate Dummy Data for other fields (as defined in the message)
        fused_msg.imu_vector = Vector3() # No IMU data, so publish an empty vector
        fused_msg.ultrasonic_distance = 0.0 # No ultrasonic data, so publish 0.0

        # Publish the fused message
        self.fused_pub.publish(fused_msg)

        # --- Debug Fused Image for RViz ---
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")

            # Fake projection of LiDAR points onto the image
            # This requires knowledge of camera intrinsics and LiDAR-camera extrinsics for a real projection
            # For debug, we'll use a simple placeholder projection
            
            # Assuming lidar_msg.ranges are in meters and ordered from angle_min to angle_max
            # Assuming lidar_msg.angle_min is negative and angle_max is positive (e.g., -pi/2 to pi/2)
            
            angle = lidar_msg.angle_min
            for r in lidar_msg.ranges:
                if r > lidar_msg.range_min and r < lidar_msg.range_max: # Filter valid ranges
                    # Convert polar to Cartesian (x, y) in LiDAR frame
                    # For a simple front-facing projection, assume x is forward, y is left/right
                    x_lidar = r * np.cos(angle)
                    y_lidar = r * np.sin(angle)

                    # Simple mapping to image coordinates (u, v)
                    # This is a very basic placeholder. Real projection needs calibration.
                    # Adjust scaling and offset as needed for your specific setup
                    u = int(cv_image.shape[1] / 2 + y_lidar * 100) # Scale y_lidar to image width
                    v = int(cv_image.shape[0] / 2 - x_lidar * 100) # Scale x_lidar to image height

                    if 0 <= u < cv_image.shape[1] and 0 <= v < cv_image.shape[0]:
                        cv2.circle(cv_image, (u, v), 2, (0, 255, 0), -1) # Draw green circle

                angle += lidar_msg.angle_increment

            # Convert OpenCV image back to ROS Image message
            fused_debug_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            fused_debug_image_msg.header = camera_msg.header # Use camera header for timestamp and frame_id
            self.fused_image_pub.publish(fused_debug_image_msg)

        except Exception as e:
            rospy.logerr(f"Error creating or publishing fused debug image: {e}")

    def run(self):
        # Keep the node alive
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SensorFusionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
