from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
from ultralytics import YOLO


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # (x1, y1, x2, y2)


class YOLODetector:
    """Lightweight wrapper around ultralytics YOLO models."""

    def __init__(self, model_path: str = "yolov8n.pt", device: str = "auto") -> None:
        self.model = YOLO(model_path)
        self.device = device

    def infer(self, frame: np.ndarray) -> List[Detection]:
        results = self.model.predict(frame, verbose=False, device=self.device, imgsz=max(frame.shape[:2]))
        detections: List[Detection] = []
        if not results:
            return detections

        names = self.model.names
        first = results[0]
        boxes = first.boxes
        for idx in range(len(boxes)):
            xyxy = boxes.xyxy[idx].tolist()
            cls_idx = int(boxes.cls[idx].item()) if boxes.cls is not None else -1
            conf = float(boxes.conf[idx].item()) if boxes.conf is not None else 0.0
            label = names.get(cls_idx, str(cls_idx)) if isinstance(names, dict) else str(cls_idx)
            detections.append(Detection(label=label, confidence=conf, bbox=tuple(xyxy)))
        return detections
