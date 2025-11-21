#!/usr/bin/env python3
"""Display lightweight live overlay with actuator values."""

from __future__ import annotations

from typing import Optional

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

from scooter_control.msg import ActuatorCommand
from sensor_msgs.msg import Image


class LiveOverlayGUI:
    """Simple GUI for overlay stream and actuator telemetry."""

    def __init__(self) -> None:
        self.overlay_topic = rospy.get_param("~overlay_topic", "/planner/overlay")
        self.command_topic = rospy.get_param("~command_topic", "/controller/actuator_values")
        self.window_name = rospy.get_param("~window_name", "Scooter Live View")
        self.bridge = CvBridge()
        self._last_cmd: Optional[ActuatorCommand] = None

        rospy.Subscriber(self.overlay_topic, Image, self._handle_overlay, queue_size=1)
        rospy.Subscriber(self.command_topic, ActuatorCommand, self._handle_cmd, queue_size=1)
        rospy.on_shutdown(self._cleanup)
        rospy.loginfo("live_overlay_gui watching %s and %s", self.overlay_topic, self.command_topic)

    def _handle_cmd(self, msg: ActuatorCommand) -> None:
        self._last_cmd = msg

    def _handle_overlay(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "GUI failed to decode overlay: %s", exc)
            return

        if self._last_cmd:
            text = (
                f"Throttle {self._last_cmd.throttle:.2f}  "
                f"Brake {self._last_cmd.brake:.2f}  "
                f"Steer {self._last_cmd.steering:.2f}"
            )
            cv2.putText(frame, text, (10, frame.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(frame, text, (10, frame.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def _cleanup(self) -> None:
        try:
            cv2.destroyWindow(self.window_name)
        except cv2.error:
            pass


def main() -> None:
    rospy.init_node("live_overlay_gui", anonymous=False)
    LiveOverlayGUI()
    rospy.spin()


if __name__ == "__main__":  # pragma: no cover
    main()
