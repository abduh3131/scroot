#!/usr/bin/env python3
"""Fuse SensorHub data, run YOLO, and publish drive commands + overlays."""

from __future__ import annotations

import importlib.util
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from sensor_interface.msg import SensorHub
from scooter_control.msg import ActuatorCommand, Detection, DetectionArray

_ultra_spec = importlib.util.find_spec("ultralytics")
if _ultra_spec is not None:  # pragma: no cover - optional dependency on Jetson
    from ultralytics import YOLO
else:  # pragma: no cover - CPU-only environments
    YOLO = None


@dataclass
class DetectionResult:
    label: str
    class_id: int
    confidence: float
    bbox: Tuple[float, float, float, float]


class LightweightYolo:
    """Small wrapper for Ultralytics YOLO with graceful fallbacks."""

    def __init__(self, weights: Path, conf: float, iou: float, device: str) -> None:
        self.weights = weights
        self.conf = conf
        self.iou = iou
        self.device = device
        self._model = None
        self._load()

    def _load(self) -> None:
        if YOLO is None:
            rospy.logwarn_once("ultralytics not installed; planner will skip camera detections.")
            return
        if not self.weights.exists():
            rospy.logwarn("YOLO weights %s not found; continuing without camera detections", self.weights)
            return
        self._model = YOLO(str(self.weights))
        rospy.loginfo("Loaded YOLO weights: %s", self.weights)

    def infer(self, frame: np.ndarray, imgsz: int) -> List[DetectionResult]:
        if self._model is None:
            return []
        try:
            results = self._model.predict(
                source=frame,
                imgsz=imgsz,
                conf=self.conf,
                iou=self.iou,
                verbose=False,
                device=self.device,
                stream=False,
            )
        except Exception as exc:  # pragma: no cover - TensorRT/PyTorch runtime faults
            rospy.logerr_throttle(1.0, "YOLO inference failed: %s", exc)
            return []

        detections: List[DetectionResult] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                try:
                    cls_id = int(box.cls.item())
                    conf = float(box.conf.item())
                    x_min, y_min, x_max, y_max = box.xyxy.cpu().numpy().flatten()
                    label = result.names.get(cls_id, f"class_{cls_id}") if hasattr(result, "names") else str(cls_id)
                    detections.append(
                        DetectionResult(
                            label=label,
                            class_id=cls_id,
                            confidence=conf,
                            bbox=(float(x_min), float(y_min), float(x_max), float(y_max)),
                        )
                    )
                except Exception:
                    continue
        return detections


class SensorFusionPlannerNode:
    """Subscribe to SensorHub, compute drive targets, and publish overlays."""

    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.sensor_topic = rospy.get_param("~sensor_topic", "/sensor_hub/data")
        self.command_topic = rospy.get_param("~command_topic", "/controller/command")
        self.overlay_topic = rospy.get_param("~overlay_topic", "/planner/overlay")
        self.status_topic = rospy.get_param("~status_topic", "/planner/status")
        self.detections_topic = rospy.get_param("~detections_topic", "/planner/detections")

        self.base_speed = float(rospy.get_param("~base_speed", 2.5))
        self.caution_distance = float(rospy.get_param("~caution_distance", 2.5))
        self.stop_distance = float(rospy.get_param("~stop_distance", 1.0))
        self.front_span = float(rospy.get_param("~front_span", 0.35))
        self.max_steering = float(rospy.get_param("~max_steering", 0.65))
        self.imgsz = int(rospy.get_param("~imgsz", 480))

        weights = Path(rospy.get_param("~yolo_weights", "~/models/yolov8n.pt")).expanduser()
        conf = float(rospy.get_param("~confidence", 0.3))
        iou = float(rospy.get_param("~iou", 0.45))
        device = rospy.get_param("~device", "cuda:0")
        self.detector = LightweightYolo(weights, conf, iou, device)

        self.command_pub = rospy.Publisher(self.command_topic, ActuatorCommand, queue_size=10)
        self.overlay_pub = rospy.Publisher(self.overlay_topic, Image, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)
        self.detections_pub = rospy.Publisher(self.detections_topic, DetectionArray, queue_size=10)

        rospy.Subscriber(self.sensor_topic, SensorHub, self._handle_sensor, queue_size=1)
        rospy.loginfo("sensor_fusion_planner_node listening to %s", self.sensor_topic)

    def _handle_sensor(self, msg: SensorHub) -> None:
        frame = self._to_bgr(msg.camera_frame)
        ranges = np.array(msg.lidar_ranges, dtype=np.float32)
        angle_min = msg.lidar_angle_min
        angle_increment = msg.lidar_angle_increment or 0.0
        angle_max = msg.lidar_angle_max

        detections = self.detector.infer(frame, self.imgsz) if frame.size else []
        detection_msg = self._to_detection_array(msg.header.stamp, detections)
        if detection_msg.detections:
            self.detections_pub.publish(detection_msg)

        front_clearance = self._sector_min(ranges, angle_min, angle_increment, angle_max, 0.0, self.front_span)
        left_clearance = self._sector_min(ranges, angle_min, angle_increment, angle_max, math.pi / 4.0, self.front_span)
        right_clearance = self._sector_min(ranges, angle_min, angle_increment, angle_max, -math.pi / 4.0, self.front_span)

        throttle, brake, steering, note = self._plan(front_clearance, left_clearance, right_clearance)

        command = ActuatorCommand()
        command.header = msg.header
        command.throttle = throttle
        command.brake = brake
        command.steering = steering
        self.command_pub.publish(command)

        overlay = self._draw_overlay(frame.copy(), detections, command, front_clearance, left_clearance, right_clearance, note)
        self._publish_overlay(overlay, msg.header)

        status = String()
        status.data = (
            f"front={front_clearance:.2f}m left={left_clearance:.2f}m right={right_clearance:.2f}m "
            f"throttle={throttle:.2f} brake={brake:.2f} steer={steering:.2f} mode={note}"
        )
        self.status_pub.publish(status)

        rospy.loginfo_throttle(1.0, status.data)

    def _to_bgr(self, img_msg: Image) -> np.ndarray:
        if img_msg is None:
            return np.zeros((0, 0, 3), dtype=np.uint8)
        try:
            return self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert camera frame: %s", exc)
            return np.zeros((0, 0, 3), dtype=np.uint8)

    def _to_detection_array(self, stamp: rospy.Time, detections: Sequence[DetectionResult]) -> DetectionArray:
        arr = DetectionArray()
        arr.header.stamp = stamp
        arr.detections = [
            Detection(
                label=det.label,
                class_id=det.class_id,
                confidence=float(det.confidence),
                x_min=float(det.bbox[0]),
                y_min=float(det.bbox[1]),
                x_max=float(det.bbox[2]),
                y_max=float(det.bbox[3]),
            )
            for det in detections
        ]
        return arr

    def _sector_min(
        self,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        angle_max: float,
        center: float,
        span: float,
    ) -> float:
        if not ranges.size or angle_increment == 0.0:
            return float("inf")
        angles = angle_min + np.arange(ranges.size) * angle_increment
        mask = np.logical_and(angles >= center - span, angles <= center + span)
        values = ranges[mask]
        values = values[np.isfinite(values)]
        if values.size == 0:
            return float("inf")
        return float(np.min(values))

    def _plan(self, front: float, left: float, right: float) -> Tuple[float, float, float, str]:
        clearance = min(front, left, right)
        if clearance == float("inf"):
            clearance = self.caution_distance

        scale = (clearance - self.stop_distance) / max(self.caution_distance - self.stop_distance, 1e-3)
        speed_scale = float(np.clip(scale, 0.0, 1.0))
        target_speed = self.base_speed * speed_scale
        throttle = float(np.clip(target_speed / self.base_speed, 0.0, 1.0)) if speed_scale > 0 else 0.0
        brake = 0.0
        note = "Cruise"

        if front < self.stop_distance:
            throttle = 0.0
            brake = float(np.clip((self.stop_distance - front) / self.stop_distance, 0.0, 1.0))
            note = "Obstacle ahead"
        elif front < self.caution_distance:
            note = "Caution zone"

        bias = right - left
        steer_scale = np.clip(bias / max(max(left, right), 1e-3), -1.0, 1.0)
        steering = float(np.clip(steer_scale * self.max_steering, -1.0, 1.0))
        if abs(steering) < 0.05:
            steering = 0.0
        return throttle, brake, steering, note

    def _draw_overlay(
        self,
        frame: np.ndarray,
        detections: Sequence[DetectionResult],
        command: ActuatorCommand,
        front: float,
        left: float,
        right: float,
        note: str,
    ) -> np.ndarray:
        if frame.size == 0:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        for det in detections:
            x_min, y_min, x_max, y_max = map(int, det.bbox)
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 200, 0), 2)
            label = f"{det.label}:{det.confidence:.2f}"
            cv2.putText(frame, label, (x_min, max(15, y_min - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        h, w = frame.shape[:2]
        overlay = frame
        hud_lines = [
            f"Throttle: {command.throttle:.2f}",
            f"Brake: {command.brake:.2f}",
            f"Steering: {command.steering:.2f}",
            f"Front clr: {front:.2f} m",
            f"Left clr: {left:.2f} m",
            f"Right clr: {right:.2f} m",
            f"Mode: {note}",
        ]
        y = 24
        for line in hud_lines:
            cv2.putText(overlay, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(overlay, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y += 24

        center = (w // 2, h - 40)
        length = int(80 * (1.0 - command.brake) + 40 * command.brake)
        angle = -command.steering * 0.5
        end_point = (int(center[0] + length * math.sin(angle)), int(center[1] - length * math.cos(angle)))
        cv2.arrowedLine(overlay, center, end_point, (0, 180, 255), 4, tipLength=0.2)
        cv2.putText(
            overlay,
            "Direction",
            (center[0] - 50, center[1] + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

        bar_w = 18
        bar_h = 120
        margin = 10
        self._draw_bar(overlay, (w - bar_w - margin, margin), bar_w, bar_h, command.throttle, (60, 220, 60), "Throttle")
        self._draw_bar(overlay, (w - 2 * bar_w - 2 * margin, margin), bar_w, bar_h, command.brake, (50, 50, 230), "Brake")
        return overlay

    def _draw_bar(self, img: np.ndarray, origin: Tuple[int, int], width: int, height: int, value: float, color, label: str) -> None:
        x, y = origin
        cv2.rectangle(img, (x, y), (x + width, y + height), (40, 40, 40), 2)
        filled_height = int(height * np.clip(value, 0.0, 1.0))
        cv2.rectangle(img, (x, y + height - filled_height), (x + width, y + height), color, -1)
        cv2.putText(img, label, (x - 10, y + height + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    def _publish_overlay(self, frame: np.ndarray, header) -> None:
        if not self.overlay_pub.get_num_connections():
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header = header
            self.overlay_pub.publish(msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to publish overlay: %s", exc)


def main() -> None:
    rospy.init_node("sensor_fusion_planner_node", anonymous=False)
    SensorFusionPlannerNode()
    rospy.spin()


if __name__ == "__main__":  # pragma: no cover
    main()
