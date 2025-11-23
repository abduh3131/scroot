from __future__ import annotations

import json
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional


@dataclass
class ActuatorCommand:
    """Simple steering/throttle/brake command normalized for common MCUs."""

    steering: float  # -1 (left) .. +1 (right)
    throttle: float  # 0 (idle) .. 1 (full)
    brake: float  # 0 (released) .. 1 (full)

    def clipped(self) -> "ActuatorCommand":
        return ActuatorCommand(
            steering=float(max(-1.0, min(1.0, self.steering))),
            throttle=float(max(0.0, min(1.0, self.throttle))),
            brake=float(max(0.0, min(1.0, self.brake))),
        )


class ActuatorPublisher:
    """Publishes actuator commands to disk and (optionally) a ROS topic.

    A JSONL log is always written when *log_path* is provided. If a ROS topic
    is supplied and the `rospy` package is available, commands are also
    published as a `std_msgs/Float32MultiArray` containing
    `[steering, throttle, brake]`.
    """

    def __init__(self, log_path: Optional[Path] = None, ros_topic: Optional[str] = None) -> None:
        self._log_path = Path(log_path) if log_path else None
        self._ros_topic = ros_topic
        self._ros_publisher = None
        if self._ros_topic:
            try:
                import rospy  # type: ignore
                from std_msgs.msg import Float32MultiArray  # type: ignore

                if not rospy.core.is_initialized():
                    rospy.init_node("simple_pilot_actuators", anonymous=True, disable_signals=True)
                self._ros_publisher = rospy.Publisher(self._ros_topic, Float32MultiArray, queue_size=10)
                self._ros_msg_type = Float32MultiArray
            except Exception:
                # ROS is optional; skip if unavailable.
                self._ros_publisher = None
                self._ros_topic = None

        if self._log_path:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            # Touch the file to ensure it exists for tailing.
            self._log_path.touch(exist_ok=True)

    def publish(self, command: ActuatorCommand) -> None:
        cmd = command.clipped()
        if self._log_path:
            payload = {"ts": time.time(), **asdict(cmd)}
            with self._log_path.open("a", encoding="utf-8") as fp:
                fp.write(json.dumps(payload) + "\n")

        if self._ros_publisher:
            msg = self._ros_msg_type()
            msg.data = [cmd.steering, cmd.throttle, cmd.brake]
            try:
                self._ros_publisher.publish(msg)
            except Exception:
                # If ROS transport fails we still keep logging to disk.
                pass
