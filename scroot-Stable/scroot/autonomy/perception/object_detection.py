from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Iterable, List, Optional

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise ImportError(
        "The 'ultralytics' package is required for object detection. Install it with 'pip install ultralytics'."
    ) from exc

from autonomy.utils.data_structures import DetectedObject, PerceptionSummary


LOGGER = logging.getLogger(__name__)


@dataclass
class ObjectDetectorConfig:
    model_name: str = "yolov8n.pt"
    confidence_threshold: float = 0.3
    iou_threshold: float = 0.4
    device: str = "auto"


class ObjectDetector:
    """Thin wrapper around the Ultralytics YOLO models."""

    def __init__(self, config: Optional[ObjectDetectorConfig] = None) -> None:
        self.config = config or ObjectDetectorConfig()
        requested_device = (self.config.device or "auto").strip().lower()
        self._requested_device = requested_device or "auto"
        self._device = "auto"
        self._model = YOLO(self.config.model_name)

        target_device = self._resolve_device(self._requested_device)
        if target_device:
            try:
                self._model.to(target_device)
                self._device = target_device
                if self._requested_device == "quadro_p520" and target_device.startswith("cuda"):
                    LOGGER.info("Quadro P520 compatibility mode active; forcing YOLO to %s.", target_device)
            except Exception as exc:  # pragma: no cover - device availability depends on runtime
                LOGGER.warning(
                    "Unable to move YOLO model to %s (%s). Falling back to CPU mode.",
                    target_device,
                    exc,
                )
                self._model.to("cpu")
                self._device = "cpu"

        self._model.fuse()

    def detect(self, frame: np.ndarray) -> PerceptionSummary:
        predict_kwargs = {
            "source": frame,
            "conf": self.config.confidence_threshold,
            "iou": self.config.iou_threshold,
            "verbose": False,
        }
        if self._device != "auto":
            predict_kwargs["device"] = self._device
        results = self._model.predict(**predict_kwargs)[0]

        detections: List[DetectedObject] = []
        names = self._model.names

        for box in results.boxes:
            cls_idx = int(box.cls.item())
            conf = float(box.conf.item())
            xyxy = box.xyxy[0].tolist()
            x_min, y_min, x_max, y_max = xyxy
            area = float((x_max - x_min) * (y_max - y_min))
            detections.append(
                DetectedObject(
                    label=str(names.get(cls_idx, cls_idx)),
                    confidence=conf,
                    bbox=(x_min, y_min, x_max, y_max),
                    area=area,
                )
            )

        return PerceptionSummary(objects=detections, frame_size=(frame.shape[1], frame.shape[0]))

    @staticmethod
    def draw_detections(frame: np.ndarray, detections: Iterable[DetectedObject]) -> np.ndarray:
        visual = frame.copy()
        for detection in detections:
            x_min, y_min, x_max, y_max = map(int, detection.bbox)
            cv2.rectangle(visual, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            label = f"{detection.label}:{detection.confidence:.2f}"
            cv2.putText(visual, label, (x_min, max(20, y_min - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return visual

    @staticmethod
    def _resolve_device(requested: str) -> Optional[str]:
        """Translate friendly device names into torch/Ultralytics device strings."""

        if requested in ("", "auto", None):  # type: ignore[comparison-overlap]
            return None
        if requested in {"cpu", "cuda"}:
            return requested
        if requested == "quadro_p520":
            return "cuda:0"
        return requested
