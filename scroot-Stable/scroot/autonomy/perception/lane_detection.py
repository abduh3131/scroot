"""Advanced lane detection inspired by openpilot's lateral planner."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from autonomy.utils.data_structures import LaneEstimate, LaneType, VehicleEnvelope


@dataclass
class LaneDetectorConfig:
    enabled: bool = True
    roi_top_ratio: float = 0.58
    min_pixels: int = 500
    smoothing: float = 0.6
    min_confidence: float = 0.35


class LaneDetector:
    """Detects lane boundaries using perspective warps and sliding windows."""

    _SRC_POINTS = np.float32([
        (0.12, 1.0),
        (0.88, 1.0),
        (0.62, 0.62),
        (0.38, 0.62),
    ])
    _DST_POINTS = np.float32([
        (0.2, 1.0),
        (0.8, 1.0),
        (0.8, 0.0),
        (0.2, 0.0),
    ])

    def __init__(
        self,
        vehicle: VehicleEnvelope,
        config: Optional[LaneDetectorConfig] = None,
    ) -> None:
        self.vehicle = vehicle
        self.config = config or LaneDetectorConfig()
        self._prev_left_fit: Optional[np.ndarray] = None
        self._prev_right_fit: Optional[np.ndarray] = None

    def analyze(self, frame: np.ndarray) -> Optional[LaneEstimate]:
        if not self.config.enabled:
            return None

        height, width = frame.shape[:2]
        if height < 200 or width < 200:
            return None

        binary = self._preprocess(frame)
        warped, Minv = self._warp(binary)
        if warped is None or Minv is None:
            return None

        search = self._sliding_window_search(warped)
        if search is None:
            return None

        left_fit, right_fit, nonzerox, nonzeroy, left_inds, right_inds = search
        left_fit, right_fit = self._smooth_fits(left_fit, right_fit)

        lane_pixels = len(left_inds) + len(right_inds)
        confidence = min(1.0, lane_pixels / max(self.config.min_pixels * 2, 1))
        if confidence < self.config.min_confidence:
            return None

        ploty = np.linspace(0, warped.shape[0] - 1, num=25)
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

        lane_points_left = self._project_points(left_fitx, ploty, Minv)
        lane_points_right = self._project_points(right_fitx, ploty, Minv)

        lane_metrics = self._compute_metrics(
            warped,
            left_fit,
            right_fit,
            nonzerox,
            nonzeroy,
            left_inds,
            right_inds,
        )
        if lane_metrics is None:
            return None

        curvature_m, offset_m, lane_width_m, recommended_bias = lane_metrics
        lane_type = self._classify_lane(lane_width_m)
        confidence = float(min(1.0, confidence * (1.0 - abs(recommended_bias) * 0.25)))

        return LaneEstimate(
            lane_type=lane_type,
            confidence=confidence,
            curvature_m=curvature_m,
            lateral_offset_m=offset_m,
            lane_width_m=lane_width_m,
            recommended_bias=recommended_bias,
            points_left=lane_points_left,
            points_right=lane_points_right,
        )

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1]
        s_channel = hls[:, :, 2]

        _, white_mask = cv2.threshold(l_channel, 200, 255, cv2.THRESH_BINARY)
        yellow_mask = cv2.inRange(hls, (15, 60, 120), (35, 204, 255))

        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        abs_sobel = np.absolute(sobelx)
        scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel)) if abs_sobel.max() > 0 else np.uint8(abs_sobel)
        _, grad_mask = cv2.threshold(scaled_sobel, 40, 255, cv2.THRESH_BINARY)

        combined = cv2.bitwise_or(white_mask, grad_mask)
        combined = cv2.bitwise_or(combined, yellow_mask)

        mask = np.zeros_like(combined)
        start_row = int(mask.shape[0] * self.config.roi_top_ratio)
        mask[start_row:, :] = combined[start_row:, :]
        return mask

    def _warp(self, binary: np.ndarray) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        height, width = binary.shape
        src = self._SRC_POINTS.copy()
        dst = self._DST_POINTS.copy()
        src[:, 0] *= width
        src[:, 1] *= height
        dst[:, 0] *= width
        dst[:, 1] *= height

        matrix = cv2.getPerspectiveTransform(src, dst)
        matrix_inv = cv2.getPerspectiveTransform(dst, src)
        warped = cv2.warpPerspective(binary, matrix, (width, height), flags=cv2.INTER_LINEAR)
        return warped, matrix_inv

    def _sliding_window_search(
        self, warped: np.ndarray
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
        histogram = np.sum(warped[warped.shape[0] // 2 :, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        leftx_base = int(np.argmax(histogram[:midpoint]))
        rightx_base = int(np.argmax(histogram[midpoint:])) + midpoint

        nwindows = 9
        window_height = int(warped.shape[0] / nwindows)
        nonzero = warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 60
        minpix = 80

        left_lane_inds: list[np.ndarray] = []
        right_lane_inds: list[np.ndarray] = []

        for window in range(nwindows):
            win_y_low = warped.shape[0] - (window + 1) * window_height
            win_y_high = warped.shape[0] - window * window_height
            win_xleft_low = max(0, leftx_current - margin)
            win_xleft_high = min(warped.shape[1], leftx_current + margin)
            win_xright_low = max(0, rightx_current - margin)
            win_xright_high = min(warped.shape[1], rightx_current + margin)

            good_left_inds = (
                (nonzeroy >= win_y_low)
                & (nonzeroy < win_y_high)
                & (nonzerox >= win_xleft_low)
                & (nonzerox < win_xleft_high)
            )
            good_right_inds = (
                (nonzeroy >= win_y_low)
                & (nonzeroy < win_y_high)
                & (nonzerox >= win_xright_low)
                & (nonzerox < win_xright_high)
            )

            left_lane_inds.append(good_left_inds.nonzero()[0])
            right_lane_inds.append(good_right_inds.nonzero()[0])

            if good_left_inds.sum() > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if good_right_inds.sum() > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_inds = np.concatenate(left_lane_inds) if left_lane_inds else np.array([])
        right_inds = np.concatenate(right_lane_inds) if right_lane_inds else np.array([])

        if len(left_inds) < self.config.min_pixels or len(right_inds) < self.config.min_pixels:
            return None

        leftx = nonzerox[left_inds]
        lefty = nonzeroy[left_inds]
        rightx = nonzerox[right_inds]
        righty = nonzeroy[right_inds]

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        return left_fit, right_fit, nonzerox, nonzeroy, left_inds, right_inds

    def _smooth_fits(self, left_fit: np.ndarray, right_fit: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        alpha = max(0.0, min(1.0, self.config.smoothing))
        if self._prev_left_fit is not None:
            left_fit = alpha * self._prev_left_fit + (1.0 - alpha) * left_fit
        if self._prev_right_fit is not None:
            right_fit = alpha * self._prev_right_fit + (1.0 - alpha) * right_fit

        self._prev_left_fit = left_fit
        self._prev_right_fit = right_fit
        return left_fit, right_fit

    def _project_points(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        matrix_inv: np.ndarray,
    ) -> Tuple[Tuple[int, int], ...]:
        pts = np.vstack((xs, ys)).T.astype(np.float32)
        pts = pts.reshape(-1, 1, 2)
        warped = cv2.perspectiveTransform(pts, matrix_inv)
        coords = tuple((int(x), int(y)) for [[x, y]] in warped)
        return coords

    def _compute_metrics(
        self,
        warped: np.ndarray,
        left_fit: np.ndarray,
        right_fit: np.ndarray,
        nonzerox: np.ndarray,
        nonzeroy: np.ndarray,
        left_inds: np.ndarray,
        right_inds: np.ndarray,
    ) -> Optional[Tuple[float, float, float, float]]:
        y_eval = warped.shape[0] - 1
        left_x = left_fit[0] * y_eval**2 + left_fit[1] * y_eval + left_fit[2]
        right_x = right_fit[0] * y_eval**2 + right_fit[1] * y_eval + right_fit[2]
        lane_width_px = float(abs(right_x - left_x))
        if lane_width_px <= 1.0:
            return None

        meters_per_px = self.vehicle.meters_per_pixel_horizontal()
        if meters_per_px <= 0.0:
            meters_per_px = self.vehicle.width_m / max(self.vehicle.calibration_reference_pixels, 1.0)
        lane_width_m = lane_width_px * meters_per_px

        vehicle_center = warped.shape[1] / 2.0
        lane_center = (left_x + right_x) / 2.0
        offset_px = vehicle_center - lane_center
        lateral_offset_m = offset_px * meters_per_px
        recommended_bias = float(np.clip(offset_px / max(lane_width_px / 2.0, 1.0), -1.0, 1.0))

        ym_per_pix = self.vehicle.calibration_reference_distance_m / max(warped.shape[0], 1.0)
        left_fit_cr = np.polyfit(nonzeroy[left_inds] * ym_per_pix, nonzerox[left_inds] * meters_per_px, 2)
        right_fit_cr = np.polyfit(nonzeroy[right_inds] * ym_per_pix, nonzerox[right_inds] * meters_per_px, 2)

        y_eval_m = y_eval * ym_per_pix
        curvature_left = self._radius_of_curvature(left_fit_cr, y_eval_m)
        curvature_right = self._radius_of_curvature(right_fit_cr, y_eval_m)
        curvature_m = float(np.mean([curvature_left, curvature_right])) if curvature_left and curvature_right else 0.0

        return curvature_m, lateral_offset_m, lane_width_m, recommended_bias

    @staticmethod
    def _radius_of_curvature(poly: np.ndarray, y_eval_m: float) -> Optional[float]:
        if poly is None or len(poly) != 3:
            return None
        A, B, _ = poly
        denominator = abs(2 * A)
        if denominator < 1e-6:
            return None
        return (1 + (2 * A * y_eval_m + B) ** 2) ** 1.5 / denominator

    @staticmethod
    def _classify_lane(lane_width_m: float) -> LaneType:
        if lane_width_m <= 0:
            return LaneType.UNKNOWN
        if lane_width_m < 1.6:
            return LaneType.BIKE_LANE
        if lane_width_m < 2.8:
            return LaneType.ROAD_EDGE
        return LaneType.SIDEWALK
