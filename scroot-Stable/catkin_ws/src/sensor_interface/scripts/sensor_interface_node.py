#!/usr/bin/env python3
"""Fuse /usb_cam/image_raw and /scan into the /sensor_hub/data stream."""

from __future__ import annotations

import math
import threading
from typing import Optional

import rospy
from geometry_msgs.msg import Vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, LaserScan

from sensor_interface.msg import SensorHub


class SensorInterfaceNode:
    """Synchronize the camera and LiDAR topics and publish SensorHub messages."""

    def __init__(self) -> None:
        self.camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
        self.lidar_topic = rospy.get_param("~lidar_topic", "/scan")
        self.frame_id = rospy.get_param("~frame_id", "sensor_hub")
        self.queue_size = rospy.get_param("~queue_size", 10)
        self.slop = rospy.get_param("~sync_slop", 0.05)

        self.publisher = rospy.Publisher(
            "/sensor_hub/data", SensorHub, queue_size=10, latch=False
        )
        self.debug_image_pub = rospy.Publisher(
            "/sensor_hub/fused_image", Image, queue_size=1, latch=False
        )

        self._imu_vector = Vector3()
        self._imu_lock = threading.Lock()
        self._ultrasonic_distance = float("nan")
        self._last_publish_time = rospy.Time(0)

        self._await_topics()

        camera_sub = Subscriber(self.camera_topic, Image)
        lidar_sub = Subscriber(self.lidar_topic, LaserScan)

        self.synchronizer = ApproximateTimeSynchronizer(
            [camera_sub, lidar_sub], queue_size=self.queue_size, slop=self.slop
        )
        self.synchronizer.registerCallback(self._handle)

        rospy.loginfo(
            "Sensor Interface Node initialized successfully. Camera=%s LiDAR=%s",
            self.camera_topic,
            self.lidar_topic,
        )

    def _await_topics(self) -> None:
        rospy.loginfo(
            "Waiting for camera topic %s and LiDAR topic %s to become available...",
            self.camera_topic,
            self.lidar_topic,
        )
        rospy.wait_for_message(self.camera_topic, Image)
        rospy.wait_for_message(self.lidar_topic, LaserScan)
        rospy.loginfo("Topics detected: %s + %s", self.camera_topic, self.lidar_topic)

    def _handle(self, camera_msg: Image, scan_msg: LaserScan) -> None:
        hub = SensorHub()
        hub.header.stamp = rospy.Time.now()
        hub.header.frame_id = self.frame_id

        hub.lidar_ranges = list(scan_msg.ranges)
        hub.lidar_angle_min = scan_msg.angle_min
        hub.lidar_angle_max = scan_msg.angle_max
        hub.lidar_angle_increment = scan_msg.angle_increment or 0.0
        hub.camera_frame = camera_msg

        with self._imu_lock:
            hub.imu_vector = self._imu_vector
            hub.ultrasonic_distance = (
                self._ultrasonic_distance if not math.isnan(self._ultrasonic_distance) else -1.0
            )

        self.publisher.publish(hub)
        if self.debug_image_pub.get_num_connections() > 0:
            self.debug_image_pub.publish(camera_msg)

        lidar_points = len(hub.lidar_ranges)
        rospy.loginfo_throttle(
            1.0,
            "Published fused /sensor_hub/data (%d LiDAR points, stamp=%.3f)",
            lidar_points,
            hub.header.stamp.to_sec(),
        )


def main() -> None:
    rospy.init_node("sensor_interface_node", anonymous=False)
    SensorInterfaceNode()
    rospy.spin()


if __name__ == "__main__":  # pragma: no cover
    main()
