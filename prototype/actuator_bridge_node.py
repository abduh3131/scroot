"""
Actuator bridge that converts normalized actuator commands into PWM duty cycles
and streams them to an Arduino-class MCU over serial. The node listens to
/scooter/actuator_cmd (geometry_msgs/Twist) produced by the navigator.

PWM layout (customize via ROS params):
- steering mapped to 1000–2000 µs servo pulse width
- throttle mapped to 1000–2000 µs ESC pulse width
- brake mapped to 0–255 where 255 requests full brake
"""

import math
import struct
from typing import Tuple

import rospy
from geometry_msgs.msg import Twist
import serial


class ActuatorBridge:
    """Translate Twist commands into PWM values for the MCU."""

    def __init__(self) -> None:
        rospy.init_node("actuator_bridge", anonymous=False)

        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)
        self.serial = serial.Serial(port, baudrate=baud, timeout=1)

        self.servo_range = self._load_range("steer_pwm_min", "steer_pwm_max", 1000, 2000)
        self.throttle_range = self._load_range("throttle_pwm_min", "throttle_pwm_max", 1000, 2000)
        self.brake_range = self._load_range("brake_pwm_min", "brake_pwm_max", 0, 255)

        rospy.Subscriber("/scooter/actuator_cmd", Twist, self._cmd_cb)
        rospy.loginfo("actuator_bridge: opened %s at %d baud", port, baud)

    def _load_range(self, lo_key: str, hi_key: str, default_lo: int, default_hi: int) -> Tuple[int, int]:
        """Read a PWM range from ROS params while ensuring it is valid."""
        lo = rospy.get_param(f"~{lo_key}", default_lo)
        hi = rospy.get_param(f"~{hi_key}", default_hi)
        if hi <= lo:
            raise ValueError(f"Invalid range for {lo_key}/{hi_key}: {lo}..{hi}")
        return lo, hi

    def _scale(self, value: float, lo: int, hi: int) -> int:
        """Clamp a normalized value and scale into the target PWM range."""
        value = max(0.0, min(1.0, value))
        return int(lo + (hi - lo) * value)

    def _cmd_cb(self, msg: Twist) -> None:
        """Convert Twist to PWM and transmit to the MCU."""
        steer_normalized = max(-1.0, min(1.0, msg.angular.z))
        steer_pwm = self._scale((steer_normalized + 1.0) / 2.0, *self.servo_range)
        throttle_pwm = self._scale(msg.linear.x, *self.throttle_range)
        brake_pwm = self._scale(msg.linear.y, *self.brake_range)

        packet = struct.pack("<HHH", steer_pwm, throttle_pwm, brake_pwm)
        self.serial.write(packet)
        self.serial.flush()

        rospy.logdebug(
            "actuator_bridge: steer=%d throttle=%d brake=%d", steer_pwm, throttle_pwm, brake_pwm
        )

    def spin(self) -> None:
        """Keep the subscriber alive."""
        rospy.spin()


if __name__ == "__main__":
    bridge = ActuatorBridge()
    bridge.spin()
