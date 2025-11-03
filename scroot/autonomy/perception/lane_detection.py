"""Simple lane detection using OpenCV heuristics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple

import cv2
import numpy as np

from autonomy.utils.data_structures import LaneObservation


@dataclass(slots=True)
class LaneDetectorConfig:
    canny_low: int = 60
    canny_high: int = 150
    hough_threshold: int = 40
    min_line_length: int = 40
    max_line_gap: int = 150
    roi_vertical_ratio: float = 0.4
    smoothing: float = 0.25


class LaneDetector:
    """Detect lanes by aggregating Hough line segments."""

    def __init__(self, config: LaneDetectorConfig | None = None) -> None:
        self.config = config or LaneDetectorConfig()
        self._last_observation: LaneObservation | None = None

    def detect(self, frame: np.ndarray) -> LaneObservation:
        if frame is None or frame.size == 0:
            return self._empty_observation()

        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, self.config.canny_low, self.config.canny_high)

        mask = np.zeros_like(edges)
        roi_top = int(height * (1.0 - self.config.roi_vertical_ratio))
        polygon = np.array(
            [
                (0, height),
                (width, height),
                (width, roi_top),
                (0, roi_top),
            ],
            dtype=np.int32,
        )
        cv2.fillPoly(mask, [polygon], 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.config.hough_threshold,
            minLineLength=self.config.min_line_length,
            maxLineGap=self.config.max_line_gap,
        )

        left_lines: list[Tuple[float, float]] = []
        right_lines: list[Tuple[float, float]] = []
        left_points: list[Tuple[int, int]] = []
        right_points: list[Tuple[int, int]] = []

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if x1 == x2:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope * x1
                if abs(slope) < 0.3:
                    continue
                if slope < 0:
                    left_lines.append((slope, intercept))
                    left_points.extend([(int(x1), int(y1)), (int(x2), int(y2))])
                else:
                    right_lines.append((slope, intercept))
                    right_points.extend([(int(x1), int(y1)), (int(x2), int(y2))])

        left_lane = self._average_line(height, left_lines)
        right_lane = self._average_line(height, right_lines)

        detected = left_lane is not None and right_lane is not None
        if not detected:
            if self._last_observation is not None:
                # return smoothed result to avoid jitter
                return LaneObservation(
                    detected=False,
                    offset_px=self._last_observation.offset_px,
                    offset_normalized=self._last_observation.offset_normalized,
                    confidence=self._last_observation.confidence * (1.0 - self.config.smoothing),
                    left_points=self._last_observation.left_points,
                    right_points=self._last_observation.right_points,
                    vanishing_point=self._last_observation.vanishing_point,
                )
            return self._empty_observation()

        left_x_top, left_x_bottom = left_lane
        right_x_top, right_x_bottom = right_lane

        lane_center_bottom = (left_x_bottom + right_x_bottom) / 2.0
        offset_px = (lane_center_bottom - width / 2.0)
        offset_norm = float(np.clip(offset_px / (width / 2.0), -1.0, 1.0))

        vanishing_y = roi_top
        vanishing_x = int((left_x_top + right_x_top) / 2.0)

        confidence = 1.0
        if left_points or right_points:
            total_points = max(1, len(left_points) + len(right_points))
            confidence = min(1.0, total_points / 20.0)

        observation = LaneObservation(
            detected=True,
            offset_px=float(offset_px),
            offset_normalized=offset_norm,
            confidence=confidence,
            left_points=tuple(left_points),
            right_points=tuple(right_points),
            vanishing_point=(vanishing_x, vanishing_y),
        )

        if self._last_observation is not None:
            observation = self._smooth(observation, self._last_observation)

        self._last_observation = observation
        return observation

    def _average_line(
        self, height: int, lines: Iterable[Tuple[float, float]]
    ) -> Tuple[int, int] | None:
        lines = list(lines)
        if not lines:
            return None
        slope = float(np.mean([s for s, _ in lines]))
        intercept = float(np.mean([b for _, b in lines]))
        y_bottom = height
        y_top = int(height * (1.0 - self.config.roi_vertical_ratio))
        x_bottom = int((y_bottom - intercept) / slope)
        x_top = int((y_top - intercept) / slope)
        return x_top, x_bottom

    def _smooth(
        self, current: LaneObservation, previous: LaneObservation
    ) -> LaneObservation:
        alpha = self.config.smoothing
        offset_px = (1 - alpha) * previous.offset_px + alpha * current.offset_px
        offset_norm = (1 - alpha) * previous.offset_normalized + alpha * current.offset_normalized
        confidence = max(current.confidence, previous.confidence * (1 - alpha))
        vanishing = current.vanishing_point or previous.vanishing_point
        return LaneObservation(
            detected=current.detected,
            offset_px=offset_px,
            offset_normalized=float(np.clip(offset_norm, -1.0, 1.0)),
            confidence=confidence,
            left_points=current.left_points or previous.left_points,
            right_points=current.right_points or previous.right_points,
            vanishing_point=vanishing,
        )

    def _empty_observation(self) -> LaneObservation:
        return LaneObservation(
            detected=False,
            offset_px=0.0,
            offset_normalized=0.0,
            confidence=0.0,
        )
