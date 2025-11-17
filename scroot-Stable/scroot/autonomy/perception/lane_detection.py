"""Advanced lane detection inspired by openpilot's lateral planner."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

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
    clahe_clip_limit: float = 2.0
    clahe_grid_size: int = 8
    morph_kernel_px: int = 5
    sobel_kernel: int = 3
    canny_threshold: Tuple[int, int] = (40, 110)
    auto_roi: bool = True
    auto_calibrate: bool = True
    calibration_smoothing: float = 0.65
    src_points: Tuple[Tuple[float, float], ...] = (
        (0.12, 1.0),
        (0.88, 1.0),
        (0.62, 0.62),
        (0.38, 0.62),
    )
    dst_points: Tuple[Tuple[float, float], ...] = (
        (0.2, 1.0),
        (0.8, 1.0),
        (0.8, 0.0),
        (0.2, 0.0),
    )


class LaneDetector:
    """Detects lane boundaries using perspective warps and sliding windows."""

    def __init__(
        self,
        vehicle: VehicleEnvelope,
        config: Optional[LaneDetectorConfig] = None,
    ) -> None:
        self.vehicle = vehicle
        self.config = config or LaneDetectorConfig()
        self._prev_left_fit: Optional[np.ndarray] = None
        self._prev_right_fit: Optional[np.ndarray] = None
        self._warp_matrix: Optional[np.ndarray] = None
        self._warp_matrix_inv: Optional[np.ndarray] = None
        self._last_shape: Optional[Tuple[int, int]] = None
        self._dynamic_src_points: Optional[Tuple[Tuple[float, float], ...]] = None
        self._dynamic_roi_ratio: Optional[float] = None
        self._latest_combined_mask: Optional[np.ndarray] = None

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

        ploty = np.linspace(0, warped.shape[0] - 1, num=40)
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

        lane_points_left = self._project_points(left_fitx, ploty, Minv)
        lane_points_right = self._project_points(right_fitx, ploty, Minv)
        lane_area = self._lane_area_polygon(lane_points_left, lane_points_right)
        birdseye_outline = self._lane_area_polygon(
            self._bev_points(left_fitx, ploty),
            self._bev_points(right_fitx, ploty),
        )

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
            lane_area=lane_area,
            birdseye_outline=birdseye_outline,
        )

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1]
        s_channel = hls[:, :, 2]

        clahe = cv2.createCLAHE(
            clipLimit=max(0.5, float(self.config.clahe_clip_limit)),
            tileGridSize=(
                max(2, int(self.config.clahe_grid_size)),
                max(2, int(self.config.clahe_grid_size)),
            ),
        )
        l_equalized = clahe.apply(l_channel)

        _, white_mask = cv2.threshold(l_equalized, 200, 255, cv2.THRESH_BINARY)
        yellow_mask = cv2.inRange(hls, (15, 60, 120), (35, 204, 255))

        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(
            gray,
            cv2.CV_64F,
            1,
            0,
            ksize=max(1, int(self.config.sobel_kernel) // 2 * 2 + 1),
        )
        abs_sobel = np.absolute(sobelx)
        scaled_sobel = (
            np.uint8(255 * abs_sobel / np.max(abs_sobel)) if abs_sobel.max() > 0 else np.uint8(abs_sobel)
        )
        _, grad_mask = cv2.threshold(scaled_sobel, 30, 255, cv2.THRESH_BINARY)

        canny = cv2.Canny(
            l_equalized,
            max(0, int(self.config.canny_threshold[0])),
            max(1, int(self.config.canny_threshold[1])),
        )

        combined = cv2.bitwise_or(white_mask, grad_mask)
        combined = cv2.bitwise_or(combined, yellow_mask)
        combined = cv2.bitwise_or(combined, canny)

        self._latest_combined_mask = combined

        roi_ratio = self.config.roi_top_ratio
        if self.config.auto_roi:
            dynamic_ratio = self._estimate_roi_top_ratio(combined)
            if dynamic_ratio is not None:
                roi_ratio = dynamic_ratio
                self._dynamic_roi_ratio = dynamic_ratio
            elif self._dynamic_roi_ratio is not None:
                roi_ratio = self._dynamic_roi_ratio

        mask = np.zeros_like(combined)
        start_row = int(mask.shape[0] * roi_ratio)
        mask[start_row:, :] = combined[start_row:, :]
        return self._refine_mask(mask)

    def _estimate_roi_top_ratio(self, mask: np.ndarray) -> Optional[float]:
        height = mask.shape[0]
        if height <= 0:
            return None
        profile = np.mean(mask > 0, axis=1)
        search_start = int(height * 0.35)
        segment = profile[search_start:]
        if segment.size == 0:
            return None
        indices = np.where(segment > 0.05)[0]
        if indices.size == 0:
            return None
        first_idx = int(indices[0]) + search_start
        ratio = float(np.clip(first_idx / max(height, 1), 0.3, 0.8))
        if self._dynamic_roi_ratio is not None:
            ratio = float(
                self.config.calibration_smoothing * self._dynamic_roi_ratio
                + (1.0 - self.config.calibration_smoothing) * ratio
            )
        return ratio

    def _refine_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel_size = max(3, int(self.config.morph_kernel_px) // 2 * 2 + 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        dilated = cv2.dilate(closed, kernel, iterations=1)
        return dilated

    def _warp(self, binary: np.ndarray) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        height, width = binary.shape
        calibration_mask = self._latest_combined_mask if self._latest_combined_mask is not None else binary
        calibration_changed = self._auto_calibrate_warp(calibration_mask)
        if self._last_shape != (height, width) or calibration_changed:
            self._compute_warp_matrices(width, height)
            self._last_shape = (height, width)

        if self._warp_matrix is None or self._warp_matrix_inv is None:
            return None, None

        warped = cv2.warpPerspective(
            binary,
            self._warp_matrix,
            (width, height),
            flags=cv2.INTER_LINEAR,
        )
        return warped, self._warp_matrix_inv

    def _compute_warp_matrices(self, width: int, height: int) -> None:
        source_points = self._dynamic_src_points or self.config.src_points
        src = self._scale_points(source_points, width, height)
        dst = self._scale_points(self.config.dst_points, width, height)
        self._warp_matrix = cv2.getPerspectiveTransform(src, dst)
        self._warp_matrix_inv = cv2.getPerspectiveTransform(dst, src)

    @staticmethod
    def _scale_points(points: Sequence[Tuple[float, float]], width: int, height: int) -> np.ndarray:
        pts = np.array(points, dtype=np.float32)
        scaled = pts.copy()
        scaled[:, 0] *= width
        scaled[:, 1] *= height
        return scaled

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

    def _auto_calibrate_warp(self, mask: np.ndarray) -> bool:
        if not self.config.auto_calibrate or mask.size == 0:
            return False

        edges = cv2.Canny(mask, 40, 150)
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=60,
            minLineLength=max(40, mask.shape[0] // 3),
            maxLineGap=40,
        )
        if lines is None:
            return False

        left_lines: list[Tuple[float, float]] = []
        right_lines: list[Tuple[float, float]] = []
        for x1, y1, x2, y2 in lines[:, 0, :]:
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.2:
                continue
            intercept = y1 - slope * x1
            if slope < 0:
                left_lines.append((slope, intercept))
            else:
                right_lines.append((slope, intercept))

        if not left_lines or not right_lines:
            return False

        left_m, left_b = np.mean(left_lines, axis=0)
        right_m, right_b = np.mean(right_lines, axis=0)

        height, width = mask.shape
        bottom_y = height - 1
        top_y = int(height * 0.55)

        def x_at_y(m: float, b: float, y_val: float) -> Optional[float]:
            if abs(m) < 1e-3:
                return None
            return (y_val - b) / m

        left_bottom = x_at_y(left_m, left_b, bottom_y)
        right_bottom = x_at_y(right_m, right_b, bottom_y)
        left_top = x_at_y(left_m, left_b, top_y)
        right_top = x_at_y(right_m, right_b, top_y)

        if None in (left_bottom, right_bottom, left_top, right_top):
            return False

        def clamp_norm(x: float) -> float:
            return float(np.clip(x / max(width - 1, 1), 0.05, 0.95))

        new_points: Tuple[Tuple[float, float], ...] = (
            (clamp_norm(left_bottom), 1.0),
            (clamp_norm(right_bottom), 1.0),
            (clamp_norm(right_top), float(np.clip(top_y / max(height - 1, 1), 0.3, 0.9))),
            (clamp_norm(left_top), float(np.clip(top_y / max(height - 1, 1), 0.3, 0.9))),
        )

        smoothing = float(np.clip(self.config.calibration_smoothing, 0.0, 0.95))
        blended_points = new_points
        if self._dynamic_src_points is not None:
            blended_points = tuple(
                (
                    old_pt[0] * smoothing + new_pt[0] * (1.0 - smoothing),
                    old_pt[1] * smoothing + new_pt[1] * (1.0 - smoothing),
                )
                for old_pt, new_pt in zip(self._dynamic_src_points, new_points)
            )

        if self._dynamic_src_points is None:
            self._dynamic_src_points = blended_points
            return True

        old = np.array(self._dynamic_src_points)
        updated = np.array(blended_points)
        if np.max(np.abs(old - updated)) < 1e-3:
            return False

        self._dynamic_src_points = tuple(map(tuple, updated.tolist()))
        return True

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

    def _lane_area_polygon(
        self,
        left: Tuple[Tuple[int, int], ...],
        right: Tuple[Tuple[int, int], ...],
    ) -> Tuple[Tuple[int, int], ...]:
        if not left or not right:
            return ()
        if len(left) != len(right):
            limit = min(len(left), len(right))
            left = left[:limit]
            right = right[:limit]
        polygon = list(left) + list(reversed(right))
        if len(polygon) < 6:
            return ()
        return tuple(polygon)

    @staticmethod
    def _bev_points(xs: np.ndarray, ys: np.ndarray) -> Tuple[Tuple[int, int], ...]:
        return tuple((int(x), int(y)) for x, y in zip(xs, ys))

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
