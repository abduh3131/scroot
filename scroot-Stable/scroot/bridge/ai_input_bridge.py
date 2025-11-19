#!/usr/bin/env python3
"""ROS bridge that feeds SensorHub messages directly into AutonomyPilot."""

from __future__ import annotations

import json
import sys
import time
from argparse import ArgumentParser
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError

try:
    from sensor_interface.msg import SensorHub
except ImportError as exc:  # pragma: no cover - requires sourced catkin workspace
    raise ImportError(
        "Unable to import sensor_interface.msg.SensorHub. Source your catkin workspace before running ai_input_bridge."
    ) from exc

from autonomy.pilot import AutonomyPilot, PilotConfig
from autonomy.utils.data_structures import ActuatorCommand, LidarSnapshot, SensorSample


class AIInputBridge:
    """Subscribes to /sensor_hub/data and runs the autonomy stack."""

    def __init__(self) -> None:
        self.bridge = CvBridge()
        self._last_log = 0.0
        self._log_interval = float(rospy.get_param("~log_interval", 1.0))
        self._mirror_runtime = bool(rospy.get_param("~mirror_runtime_inputs", True))
        self._runtime_dir = Path(rospy.get_param("~runtime_dir", str(Path.home() / "scroot" / "runtime_inputs"))).expanduser()
        if self._mirror_runtime:
            self._runtime_dir.mkdir(parents=True, exist_ok=True)
            self._camera_path = self._runtime_dir / "camera.jpg"
            self._lidar_path = self._runtime_dir / "lidar.npy"
            self._meta_path = self._runtime_dir / "sensor_meta.json"
        else:
            self._camera_path = None
            self._lidar_path = None
            self._meta_path = None

        config = self._build_pilot_config()
        rospy.loginfo(
            "Launching AutonomyPilot (model=%s, device=%s, visualize=%s, mirror_runtime=%s)",
            config.model_name,
            config.detector_device,
            config.visualize,
            self._mirror_runtime,
        )
        self._pilot = AutonomyPilot(config)
        rospy.on_shutdown(self._handle_shutdown)

        rospy.Subscriber("/sensor_hub/data", SensorHub, self._handle_sensor_hub, queue_size=1)
        rospy.loginfo("ai_input_bridge subscribed to /sensor_hub/data")

    def _build_pilot_config(self) -> PilotConfig:
        cfg = PilotConfig()
        cfg.use_internal_sensors = False
        cfg.model_name = rospy.get_param("~model", cfg.model_name)
        cfg.confidence_threshold = float(rospy.get_param("~confidence", cfg.confidence_threshold))
        cfg.iou_threshold = float(rospy.get_param("~iou", cfg.iou_threshold))
        cfg.detector_device = rospy.get_param("~device", cfg.detector_device)
        cfg.visualize = bool(rospy.get_param("~visualize", cfg.visualize))
        command_file = rospy.get_param("~command_file", "")
        if command_file:
            cfg.command_file = Path(command_file).expanduser()
        persona = rospy.get_param("~persona", cfg.companion_persona)
        cfg.companion_persona = persona
        advisor = rospy.get_param("~advisor", "on")
        cfg.companion_enabled = advisor.lower() != "off"
        return cfg

    def _handle_shutdown(self) -> None:
        try:
            self._pilot.stop()
        except Exception:  # pragma: no cover - defensive during shutdown
            pass

    def _handle_sensor_hub(self, msg: SensorHub) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg.camera_frame, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("Failed to convert camera frame: %s", exc)
            return
        except Exception as exc:  # pragma: no cover - runtime safety
            rospy.logwarn("Unexpected camera conversion error: %s", exc)
            return

        lidar_snapshot = self._build_lidar_snapshot(msg)
        timestamp = 0.0
        if msg.header and msg.header.stamp:
            timestamp = msg.header.stamp.to_sec()
        else:
            timestamp = rospy.Time.now().to_sec()

        sample = SensorSample(frame=frame, timestamp=timestamp, lidar=lidar_snapshot)
        command = self._pilot.process_sample(sample)

        if self._mirror_runtime:
            self._mirror_runtime_inputs(frame, lidar_snapshot, msg, timestamp)

        self._throttled_log(command, lidar_snapshot)

    def _build_lidar_snapshot(self, msg: SensorHub) -> Optional[LidarSnapshot]:
        try:
            ranges = np.asarray(msg.lidar_ranges, dtype=np.float32)
        except Exception as exc:  # pragma: no cover - sensor edge case
            rospy.logwarn("Failed to parse LiDAR ranges: %s", exc)
            return None
        imu_tuple = self._vector3_to_tuple(getattr(msg, "imu_vector", None))
        ultrasonic = float(getattr(msg, "ultrasonic_distance", -1.0))
        if ultrasonic <= 0.0:
            ultrasonic = None
        stamp = msg.header.stamp.to_sec() if msg.header and msg.header.stamp else rospy.Time.now().to_sec()
        return LidarSnapshot(
            ranges=ranges,
            angle_min=float(getattr(msg, "lidar_angle_min", 0.0)),
            angle_max=float(getattr(msg, "lidar_angle_max", 0.0)),
            angle_increment=float(getattr(msg, "lidar_angle_increment", 0.0)),
            stamp=stamp,
            imu_vector=imu_tuple,
            ultrasonic_distance=ultrasonic,
        )

    def _vector3_to_tuple(self, vector: Optional[object]) -> Optional[tuple[float, float, float]]:
        if vector is None:
            return None
        try:
            return (float(vector.x), float(vector.y), float(vector.z))
        except AttributeError:
            return None

    def _mirror_runtime_inputs(
        self,
        frame: np.ndarray,
        lidar_snapshot: Optional[LidarSnapshot],
        msg: SensorHub,
        timestamp: float,
    ) -> None:
        if self._camera_path is not None:
            try:
                cv2.imwrite(str(self._camera_path), frame)
            except Exception as exc:  # pragma: no cover - runtime safety
                rospy.logwarn("Failed to write camera snapshot: %s", exc)
        if self._lidar_path is not None and lidar_snapshot is not None:
            try:
                np.save(self._lidar_path, lidar_snapshot.ranges, allow_pickle=False)
            except Exception as exc:  # pragma: no cover - runtime safety
                rospy.logwarn("Failed to mirror LiDAR ranges: %s", exc)
        if self._meta_path is not None:
            metadata = {
                "stamp": timestamp,
                "camera_stamp": self._camera_stamp(msg),
                "lidar_angle_min": float(getattr(msg, "lidar_angle_min", 0.0)),
                "lidar_angle_max": float(getattr(msg, "lidar_angle_max", 0.0)),
                "lidar_angle_increment": float(getattr(msg, "lidar_angle_increment", 0.0)),
                "imu_vector": {
                    "x": float(getattr(getattr(msg, "imu_vector", None), "x", 0.0)),
                    "y": float(getattr(getattr(msg, "imu_vector", None), "y", 0.0)),
                    "z": float(getattr(getattr(msg, "imu_vector", None), "z", 0.0)),
                },
                "ultrasonic_distance": float(getattr(msg, "ultrasonic_distance", -1.0)),
            }
            try:
                self._meta_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
            except Exception as exc:  # pragma: no cover - runtime safety
                rospy.logwarn("Failed to export SensorHub metadata: %s", exc)

    def _camera_stamp(self, msg: SensorHub) -> float:
        try:
            return msg.camera_frame.header.stamp.to_sec()
        except Exception:
            return 0.0

    def _throttled_log(self, command: ActuatorCommand, lidar_snapshot: Optional[LidarSnapshot]) -> None:
        now = time.time()
        if now - self._last_log < self._log_interval:
            return
        self._last_log = now
        lidar_text = "n/a"
        if lidar_snapshot is not None:
            closest = lidar_snapshot.closest_distance()
            if closest is not None:
                lidar_text = f"{closest:.2f} m"
        rospy.loginfo(
            "ai_input_bridge command steer=%+.3f throttle=%.3f brake=%.3f lidar_min=%s",
            command.steer,
            command.throttle,
            command.brake,
            lidar_text,
        )


def _parse_cli_args(argv: Optional[list[str]] = None) -> tuple[Optional[bool], Optional[str], list[str]]:
    parser = ArgumentParser(add_help=False)
    parser.add_argument("--mirror-runtime-inputs", dest="mirror", action="store_true")
    parser.add_argument("--no-mirror-runtime-inputs", dest="mirror", action="store_false")
    parser.add_argument("--runtime-dir", dest="runtime_dir")
    parser.set_defaults(mirror=None, runtime_dir=None)
    args, remaining = parser.parse_known_args(argv)
    return args.mirror, args.runtime_dir, remaining


def main(argv: Optional[list[str]] = None) -> None:
    mirror_override, runtime_dir_override, remaining = _parse_cli_args(argv)
    sys.argv = [sys.argv[0]] + remaining
    rospy.init_node("ai_input_bridge", anonymous=False)
    if mirror_override is not None:
        rospy.set_param("~mirror_runtime_inputs", bool(mirror_override))
    if runtime_dir_override:
        rospy.set_param("~runtime_dir", runtime_dir_override)
    AIInputBridge()
    rospy.spin()


if __name__ == "__main__":  # pragma: no cover
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        sys.exit(0)
