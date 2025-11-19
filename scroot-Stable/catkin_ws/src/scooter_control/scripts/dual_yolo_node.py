#!/usr/bin/env python3
"""Dual YOLO TensorRT node that republishes annotated detections."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from scooter_control.msg import Detection, DetectionArray

try:  # pragma: no cover - optional dependency on Jetson
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - CPU-only environments
    YOLO = None


@dataclass
class DetectionResult:
    label: str
    class_id: int
    confidence: float
    bbox: tuple[float, float, float, float]


class YOLOSession:
    """Lightweight wrapper that runs a YOLO model/engine with Ultralytics."""

    def __init__(self, model_path: Path, conf: float, iou: float, device: str) -> None:
        self.model_path = model_path
        self.conf = conf
        self.iou = iou
        self.device = device
        self._model = None
        self._load()

    def _load(self) -> None:
        if YOLO is None:
            rospy.logwarn_once(
                "ultralytics package not available; dual_yolo_node will not emit detections."
            )
            return
        if not self.model_path.exists():
            rospy.logwarn("YOLO weights %s not found; skipping this session.", self.model_path)
            return
        self._model = YOLO(str(self.model_path))
        rospy.loginfo(
            "Loaded YOLO model from %s (%s)",
            self.model_path,
            "TensorRT" if self.model_path.suffix == ".engine" else "PyTorch/ONNX",
        )

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
            if not hasattr(result, "boxes"):
                continue
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                try:
                    cls_id = int(box.cls.item())
                    conf = float(box.conf.item())
                    xyxy = box.xyxy.cpu().numpy().flatten()
                    x_min, y_min, x_max, y_max = [float(v) for v in xyxy]
                    label = result.names.get(cls_id, f"class_{cls_id}") if hasattr(result, "names") else str(cls_id)
                    detections.append(
                        DetectionResult(
                            label=label,
                            class_id=cls_id,
                            confidence=conf,
                            bbox=(x_min, y_min, x_max, y_max),
                        )
                    )
                except Exception:  # pragma: no cover - guard for tensor edge cases
                    continue
        return detections


class DualYoloDetector:
    """Runs two YOLO contexts (wide + crop) and merges their detections."""

    def __init__(
        self,
        primary_weights: Path,
        secondary_weights: Path,
        conf: float,
        iou: float,
        device: str,
        imgsz_primary: int,
        imgsz_secondary: int,
    ) -> None:
        self.primary = YOLOSession(primary_weights, conf, iou, device)
        self.secondary = YOLOSession(secondary_weights, conf, iou, device)
        self.imgsz_primary = imgsz_primary
        self.imgsz_secondary = imgsz_secondary

    def infer(self, frame: np.ndarray) -> List[DetectionResult]:
        detections = self.primary.infer(frame, self.imgsz_primary)
        if frame.size and self.secondary._model is not None:
            cropped = frame
            h, w = frame.shape[:2]
            if h > 0 and w > 0:
                inset = frame[h // 6 : h, w // 6 : w - w // 6]
                if inset.size:
                    cropped = inset
            extra = self.secondary.infer(cropped, self.imgsz_secondary)
            detections.extend(extra)
        return detections


class DualYoloNode:
    """ROS1 node that exposes annotated images and structured detections."""

    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
        primary_path = Path(rospy.get_param("~primary_engine", "~/models/yolov8n_fp16.engine")).expanduser()
        secondary_path = Path(
            rospy.get_param("~secondary_engine", "~/models/yolov8n_medium.engine")
        ).expanduser()
        fallback = Path(rospy.get_param("~fallback_model", "yolov8n.pt")).expanduser()
        if not primary_path.exists() and fallback.exists():
            primary_path = fallback
        if not secondary_path.exists() and fallback.exists():
            secondary_path = fallback
        conf = float(rospy.get_param("~confidence", 0.35))
        iou = float(rospy.get_param("~iou", 0.45))
        device = rospy.get_param("~device", "cuda:0")
        imgsz_primary = int(rospy.get_param("~imgsz_primary", 640))
        imgsz_secondary = int(rospy.get_param("~imgsz_secondary", 896))

        self.detector = DualYoloDetector(
            primary_path,
            secondary_path,
            conf,
            iou,
            device,
            imgsz_primary,
            imgsz_secondary,
        )

        self.annotated_pub = rospy.Publisher("/AI/annotated_image", Image, queue_size=1)
        self.detections_pub = rospy.Publisher("/AI/detections", DetectionArray, queue_size=10)

        rospy.Subscriber(self.camera_topic, Image, self._handle_frame, queue_size=1)
        rospy.loginfo("dual_yolo_node subscribed to %s", self.camera_topic)

    def _handle_frame(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(1.0, "Failed to convert camera frame: %s", exc)
            return
        detections = self.detector.infer(frame)
        annotated = self._draw_detections(frame.copy(), detections)
        self._publish_detections(msg.header, detections)
        self._publish_image(annotated, msg.header)

    def _draw_detections(self, frame: np.ndarray, detections: List[DetectionResult]) -> np.ndarray:
        for det in detections:
            x_min, y_min, x_max, y_max = map(int, det.bbox)
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            label = f"{det.label}:{det.confidence:.2f}"
            cv2.putText(frame, label, (x_min, max(12, y_min - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return frame

    def _publish_detections(self, header, detections: List[DetectionResult]) -> None:
        msg = DetectionArray()
        msg.header = header
        msg.detections = [
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
        self.detections_pub.publish(msg)

    def _publish_image(self, frame: np.ndarray, header) -> None:
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(1.0, "Failed to publish annotated frame: %s", exc)
            return
        ros_image.header = header
        self.annotated_pub.publish(ros_image)


def main() -> None:
    rospy.init_node("dual_yolo_node", anonymous=False)
    DualYoloNode()
    rospy.spin()


if __name__ == "__main__":  # pragma: no cover
    main()
