#!/usr/bin/env python3
"""Visualize SensorHub camera frames and LiDAR summary."""

from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_interface.msg import SensorHub


class SensorHubVisualizer:
    """Display SensorHub camera frames and log LiDAR stats."""

    def __init__(self) -> None:
        self.window_name = rospy.get_param("~window_name", "SensorHub Viewer")
        self.log_period = rospy.Duration(rospy.get_param("~log_interval", 1.0))
        self._last_log: Optional[rospy.Time] = None
        self.bridge = CvBridge()

        rospy.Subscriber(
            "/sensor_hub/data", SensorHub, self._handle_sensor_hub, queue_size=1
        )
        rospy.on_shutdown(self._cleanup)
        rospy.loginfo("visualize_sensor_hub.py waiting for /sensor_hub/data ...")

    def _handle_sensor_hub(self, msg: SensorHub) -> None:
        self._show_image(msg)
        self._log_lidar(msg)

    def _show_image(self, msg: SensorHub) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg.camera_frame, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Failed to convert SensorHub image: %s", exc)
            return

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def _log_lidar(self, msg: SensorHub) -> None:
        now = rospy.Time.now()
        if self._last_log and now - self._last_log < self.log_period:
            return
        self._last_log = now

        ranges = np.array(msg.lidar_ranges, dtype=np.float32)
        ranges = ranges[np.isfinite(ranges)]
        points = ranges.size
        if points:
            closest = float(np.min(ranges))
            median = float(np.median(ranges))
        else:
            closest = math.nan
            median = math.nan

        rospy.loginfo(
            "SensorHub stamp=%.3f LiDAR points=%d closest=%.2fm median=%.2fm",
            msg.header.stamp.to_sec(),
            points,
            closest,
            median,
        )

    def _cleanup(self) -> None:
        try:
            cv2.destroyWindow(self.window_name)
        except cv2.error:
            pass


def main() -> None:
    rospy.init_node("visualize_sensor_hub", anonymous=False)
    SensorHubVisualizer()
    rospy.spin()


if __name__ == "__main__":  # pragma: no cover
    main()
