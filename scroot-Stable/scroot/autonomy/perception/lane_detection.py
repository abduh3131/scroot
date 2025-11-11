"""Simple lane detection and visualization utilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np


LaneSegment = Tuple[int, int, int, int]


@dataclass(frozen=True)
class LaneDetectionResult:
    """Container for detected lane line segments and overlays."""

    segments: Tuple[LaneSegment, ...]
    region_overlay: np.ndarray | None


class LaneDetector:
    """Detect lane lines using Canny edges and probabilistic Hough transform."""

    def __init__(
        self,
        *,
        canny_threshold1: int = 100,
        canny_threshold2: int = 180,
        hough_threshold: int = 50,
        min_line_length_ratio: float = 0.25,
        max_line_gap_ratio: float = 0.05,
        region_height_ratio: float = 0.55,
        region_half_width_ratio: float = 0.45,
    ) -> None:
        self.canny_threshold1 = canny_threshold1
        self.canny_threshold2 = canny_threshold2
        self.hough_threshold = hough_threshold
        self.min_line_length_ratio = min_line_length_ratio
        self.max_line_gap_ratio = max_line_gap_ratio
        self.region_height_ratio = region_height_ratio
        self.region_half_width_ratio = region_half_width_ratio

    def detect(self, frame: np.ndarray) -> LaneDetectionResult:
        height, width = frame.shape[:2]
        if height == 0 or width == 0:
            return LaneDetectionResult(segments=(), region_overlay=None)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, self.canny_threshold1, self.canny_threshold2)

        polygon = self._build_region_polygon(width, height)
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, [polygon], 255)
        roi_edges = cv2.bitwise_and(edges, mask)

        min_line_length = max(20, int(width * self.min_line_length_ratio))
        max_line_gap = max(20, int(width * self.max_line_gap_ratio))

        lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=min_line_length,
            maxLineGap=max_line_gap,
        )

        segments: list[LaneSegment] = []
        line_overlay = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = map(int, line[0])
                segments.append((x1, y1, x2, y2))
                cv2.line(line_overlay, (x1, y1), (x2, y2), (0, 255, 255), 4)

        region_overlay: np.ndarray | None = None
        if segments:
            shaded = np.zeros_like(frame)
            cv2.fillPoly(shaded, [polygon], (0, 120, 255))
            region_overlay = cv2.addWeighted(shaded, 0.4, line_overlay, 0.6, 0)
        elif np.count_nonzero(mask) > 0:
            # Even if no lines are detected, show the region of interest subtly.
            shaded = np.zeros_like(frame)
            cv2.fillPoly(shaded, [polygon], (40, 40, 40))
            region_overlay = shaded

        return LaneDetectionResult(segments=tuple(segments), region_overlay=region_overlay)

    @staticmethod
    def draw_overlay(frame: np.ndarray, result: LaneDetectionResult) -> np.ndarray:
        """Blend the detected lane lines and region overlay onto the frame."""

        if result.region_overlay is None and not result.segments:
            return frame

        output = frame.copy()
        if result.region_overlay is not None:
            output = cv2.addWeighted(output, 1.0, result.region_overlay, 0.5, 0)

        for x1, y1, x2, y2 in result.segments:
            cv2.line(output, (x1, y1), (x2, y2), (0, 255, 255), 3)

        return output

    def _build_region_polygon(self, width: int, height: int) -> np.ndarray:
        center_x = width // 2
        half_width = int(width * self.region_half_width_ratio)
        top_y = int(height * (1.0 - self.region_height_ratio))
        polygon = np.array(
            [
                (0, height),
                (max(0, center_x - half_width), top_y),
                (min(width, center_x + half_width), top_y),
                (width, height),
            ],
            dtype=np.int32,
        )
        return polygon

