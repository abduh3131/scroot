#!/usr/bin/env python3
"""Controller node that streams planner commands to actuators and scripts."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import rospy

from scooter_control.msg import ActuatorCommand


class ControllerNode:
    """Apply planner commands, smooth them, and export actuator values."""

    def __init__(self) -> None:
        self.command_topic = rospy.get_param("~command_topic", "/controller/command")
        self.output_topic = rospy.get_param("~actuator_topic", "/controller/actuator_values")
        self.command_log_path = Path(rospy.get_param("~command_log_path", "~/controller_command.txt")).expanduser()
        self.smoothing = float(rospy.get_param("~smoothing", 0.2))

        self.publisher = rospy.Publisher(self.output_topic, ActuatorCommand, queue_size=10)
        rospy.Subscriber(self.command_topic, ActuatorCommand, self._handle_command, queue_size=5)
        rospy.loginfo("controller_node forwarding %s -> %s", self.command_topic, self.output_topic)

        self._last_cmd: Optional[ActuatorCommand] = None

    def _handle_command(self, msg: ActuatorCommand) -> None:
        cmd = self._smooth(msg)
        cmd.header.stamp = rospy.Time.now()
        self.publisher.publish(cmd)
        self._last_cmd = cmd
        self._write_to_script(cmd)
        rospy.loginfo_throttle(1.0, "Controller throttle=%.2f brake=%.2f steer=%.2f", cmd.throttle, cmd.brake, cmd.steering)

    def _smooth(self, cmd: ActuatorCommand) -> ActuatorCommand:
        if self._last_cmd is None:
            return cmd
        blended = ActuatorCommand()
        blended.header = cmd.header
        blended.throttle = self._blend(self._last_cmd.throttle, cmd.throttle)
        blended.brake = self._blend(self._last_cmd.brake, cmd.brake)
        blended.steering = self._blend(self._last_cmd.steering, cmd.steering)
        return blended

    def _blend(self, prev: float, new: float) -> float:
        alpha = np.clip(self.smoothing, 0.0, 1.0)
        return float((1 - alpha) * prev + alpha * new)

    def _write_to_script(self, cmd: ActuatorCommand) -> None:
        try:
            self.command_log_path.parent.mkdir(parents=True, exist_ok=True)
            text = f"THROTTLE={cmd.throttle:.3f}\nBRAKE={cmd.brake:.3f}\nSTEERING={cmd.steering:.3f}\n"
            self.command_log_path.write_text(text)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to stream actuator command to %s: %s", self.command_log_path, exc)


def main() -> None:
    rospy.init_node("controller_node", anonymous=False)
    ControllerNode()
    rospy.spin()


if __name__ == "__main__":  # pragma: no cover
    main()
