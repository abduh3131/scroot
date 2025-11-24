"""
Navigator node that consumes the bridged sensor topics and emits normalized
actuator commands. The planner is intentionally simple: it favors ambient
cruising while backing off when LiDAR detects obstacles and nudges steering away
from the densest detections.

Subscriptions:
- /scooter/lidar_scan (sensor_msgs/LaserScan)
- /scooter/yolo_detections (vision_msgs/Detection2DArray)

Publications:
- /scooter/actuator_cmd (geometry_msgs/Twist)
  - linear.x -> throttle in [0, 1]
  - linear.y -> brake in [0, 1]
  - angular.z -> steering in [-1, 1] where left is positive
"""

import math
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray


class Navigator:
    """Planner that outputs steering/throttle/brake commands."""

    def __init__(self) -> None:
        rospy.init_node("navigator", anonymous=False)

        self.min_distance: Optional[float] = None
        self.steering_bias: float = 0.0

        self.cmd_pub = rospy.Publisher("/scooter/actuator_cmd", Twist, queue_size=1)
        rospy.Subscriber("/scooter/lidar_scan", LaserScan, self._lidar_cb)
        rospy.Subscriber("/scooter/yolo_detections", Detection2DArray, self._yolo_cb)

        rospy.loginfo("navigator: ready to plan using LiDAR and YOLO detections")

    def _lidar_cb(self, msg: LaserScan) -> None:
        """Capture the closest obstacle distance from the LiDAR scan."""
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
            if valid_ranges:
                self.min_distance = min(valid_ranges)
                rospy.logdebug("navigator: min distance %.2f m", self.min_distance)

    def _yolo_cb(self, msg: Detection2DArray) -> None:
        """Compute a simple steering bias from YOLO detections."""
        if not msg.detections:
            self.steering_bias = 0.0
            return

        # Average the normalized x-centroid of all detections; bias left if objects appear on the right.
        centroids = []
        for detection in msg.detections:
            if detection.bbox.size_x <= 0:
                continue
            norm_x = detection.bbox.center.x
            image_width = detection.bbox.size_x
            # Normalize around image center: >0 objects on the right, <0 objects on the left.
            centroids.append((norm_x / image_width) - 0.5)

        if centroids:
            self.steering_bias = -sum(centroids) / len(centroids)
            self.steering_bias = max(-1.0, min(1.0, self.steering_bias))
            rospy.logdebug("navigator: steering bias %.2f", self.steering_bias)

    def _plan(self) -> Twist:
        """Create a Twist command based on the latest sensor readings."""
        cmd = Twist()
        obstacle_stop_distance = rospy.get_param("~stop_distance", 1.0)
        cruise_throttle = rospy.get_param("~cruise_throttle", 0.25)

        # Default to gentle cruising.
        cmd.linear.x = cruise_throttle
        cmd.linear.y = 0.0  # brake
        cmd.angular.z = self.steering_bias

        if self.min_distance is not None and self.min_distance < obstacle_stop_distance:
            # Fail-safe: stop and brake when something is too close.
            cmd.linear.x = 0.0
            cmd.linear.y = 1.0
            rospy.logwarn(
                "navigator: obstacle detected at %.2f m — issuing brake", self.min_distance
            )
        return cmd

    def spin(self) -> None:
        """Publish plans at a fixed rate."""
        rate_hz = rospy.get_param("~rate_hz", 10)
        rate = rospy.Rate(rate_hz)
        rospy.loginfo("navigator: publishing actuator commands at %s Hz", rate_hz)
        while not rospy.is_shutdown():
            cmd = self._plan()
            self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    navigator = Navigator()
    navigator.spin()
