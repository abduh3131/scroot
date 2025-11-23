from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np

from .actuator_output import ActuatorCommand
from .detector import Detection


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class NavigatorConfig:
    cruise_throttle: float = 0.35
    brake_area_threshold: float = 0.22  # fraction of frame covered by the largest box
    steer_gain: float = 0.8  # multiplier applied to normalized horizontal offset


class Navigator:
    """Naive obstacle-aware navigator.

    The navigator keeps a modest cruise throttle, biases steering away from the
    weighted centroid of detected objects, and taps the brake when a large
    detection blocks the center of the image.
    """

    def __init__(self, config: NavigatorConfig) -> None:
        self.config = config

    def plan(self, detections: Iterable[Detection], frame_shape: Tuple[int, int]) -> ActuatorCommand:
        height, width = frame_shape
        frame_area = float(width * height)
        if frame_area == 0:
            return ActuatorCommand(steering=0.0, throttle=0.0, brake=1.0)

        det_list = list(detections)
        if not det_list:
            return ActuatorCommand(steering=0.0, throttle=self.config.cruise_throttle, brake=0.0)

        # Compute weighted horizontal offset away from detections.
        offsets = []
        max_area_ratio = 0.0
        for det in det_list:
            x1, y1, x2, y2 = det.bbox
            center_x = 0.5 * (x1 + x2)
            normalized = (center_x - (width / 2)) / (width / 2)
            area_ratio = max(0.0, (x2 - x1) * (y2 - y1) / frame_area)
            weight = det.confidence + area_ratio
            offsets.append(weight * normalized)
            max_area_ratio = max(max_area_ratio, area_ratio)

        steer_correction = -self.config.steer_gain * float(np.tanh(sum(offsets)))
        steer_correction = _clamp(steer_correction, -1.0, 1.0)

        # Throttle tap down as the largest object fills more of the frame.
        throttle_scale = max(0.0, 1.0 - max_area_ratio)
        throttle = self.config.cruise_throttle * throttle_scale

        # Brake if something dominates the center of view.
        brake = 1.0 if max_area_ratio >= self.config.brake_area_threshold else 0.0

        return ActuatorCommand(steering=steer_correction, throttle=throttle, brake=brake)
