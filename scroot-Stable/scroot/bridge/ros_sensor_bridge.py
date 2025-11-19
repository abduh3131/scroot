#!/usr/bin/env python3
"""ROS bridge that mirrors SensorHub data into runtime input files."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError

try:
    from sensor_interface.msg import SensorHub
except ImportError as exc:  # pragma: no cover - depends on ROS workspace
    raise ImportError(
        "Unable to import sensor_interface.msg.SensorHub. Source your ROS workspace before running this bridge."
    ) from exc


class SensorHubBridge:
    """Subscribes to /sensor_hub/data and writes camera/LiDAR runtime files."""

    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir.expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.camera_path = self.output_dir / "camera.jpg"
        self.lidar_path = self.output_dir / "lidar.npy"
        self.bridge = CvBridge()
        self._last_log = 0.0
        self._log_interval = 1.0

        rospy.Subscriber("/sensor_hub/data", SensorHub, self._handle_sensor_hub, queue_size=1)
        rospy.loginfo(
            "ai_input_bridge subscribed to /sensor_hub/data; writing camera frames to %s and LiDAR ranges to %s",
            self.camera_path,
            self.lidar_path,
        )

    def _handle_sensor_hub(self, msg: SensorHub) -> None:
        lidar_count = 0
        try:
            image_msg = self._extract_image(msg)
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            cv2.imwrite(str(self.camera_path), frame)
        except CvBridgeError as exc:
            rospy.logwarn("Failed to convert camera image: %s", exc)
        except Exception as exc:  # pragma: no cover - runtime safety
            rospy.logwarn("Failed to write camera frame: %s", exc)

        try:
            ranges = self._extract_lidar_ranges(msg)
            lidar_count = int(ranges.size)
            np.save(self.lidar_path, ranges, allow_pickle=False)
        except Exception as exc:  # pragma: no cover - runtime safety
            rospy.logwarn("Failed to export LiDAR ranges: %s", exc)

        self._log_status(lidar_count)

    def _extract_image(self, msg: SensorHub):
        for field in ("camera", "image", "fused_image", "camera_frame"):
            image = getattr(msg, field, None)
            if image is not None:
                return image
        raise AttributeError("SensorHub message does not contain a camera image field")

    def _extract_lidar_ranges(self, msg: SensorHub) -> np.ndarray:
        if hasattr(msg, "lidar_ranges"):
            ranges = getattr(msg, "lidar_ranges")
            return np.asarray(ranges, dtype=np.float32)
        for field in ("lidar", "lidar_scan", "fused_lidar", "scan"):
            scan = getattr(msg, field, None)
            if scan is None:
                continue
            if hasattr(scan, "ranges"):
                return np.asarray(scan.ranges, dtype=np.float32)
        raise AttributeError("SensorHub message does not contain LiDAR ranges")

    def _log_status(self, lidar_count: int) -> None:
        now = time.monotonic()
        if now - self._last_log < self._log_interval:
            return
        self._last_log = now
        rospy.loginfo(
            "Updated runtime inputs: camera=%s lidar_points=%d",
            self.camera_path,
            lidar_count,
        )


def main() -> None:
    rospy.init_node("ai_input_bridge", anonymous=False)
    output_dir = Path.home() / "scroot" / "runtime_inputs"
    SensorHubBridge(output_dir)
    rospy.spin()


if __name__ == "__main__":  # pragma: no cover
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
