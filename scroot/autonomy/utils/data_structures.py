from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class DetectedObject:
    """Represents a single detection from the perception system."""

    label: str
    confidence: float
    bbox: Tuple[float, float, float, float]
    area: float

    @property
    def center_x(self) -> float:
        x_min, _, x_max, _ = self.bbox
        return (x_min + x_max) / 2.0

    @property
    def center_y(self) -> float:
        _, y_min, _, y_max = self.bbox
        return (y_min + y_max) / 2.0


@dataclass(frozen=True)
class HighLevelCommand:
    """Structured operator intent for the navigator and advisor."""

    command_type: str
    target: Optional[str] = None
    distance_m: Optional[float] = None
    raw_text: str = ""


@dataclass(frozen=True)
class NavigationDecision:
    """Simplified high-level plan for the controller."""

    steering_bias: float  # -1.0 (left) .. +1.0 (right)
    desired_speed: float  # meters per second
    hazard_level: float   # 0.0 (clear) .. 1.0 (imminent stop)
    metadata: Dict[str, float]
    enforced_stop: bool = False
    directive: str = ""
    caption: str = ""
    goal_context: str = ""


@dataclass(frozen=True)
class LaneObservation:
    """Summary of the most recent lane detection pass."""

    detected: bool
    offset_px: float
    offset_normalized: float
    confidence: float
    left_points: Tuple[Tuple[int, int], ...] = ()
    right_points: Tuple[Tuple[int, int], ...] = ()
    vanishing_point: Optional[Tuple[int, int]] = None


class LaneType(str, Enum):
    """Categorization of the surface the scooter is currently occupying."""

    BIKE_LANE = "BIKE_LANE"
    SIDEWALK = "SIDEWALK"
    ROAD_EDGE = "ROAD_EDGE"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class ContextSnapshot:
    """Aggregated scene context used by the safety advisor."""

    lane_type: LaneType
    confidence: float
    sensor_confidence: float
    recommended_bias: float
    metadata: Dict[str, float] = field(default_factory=dict)


class AdvisorVerdict(str, Enum):
    """Advisor arbitration outcome."""

    ALLOW = "ALLOW"
    AMEND = "AMEND"
    BLOCK = "BLOCK"


@dataclass(frozen=True)
class AdvisorReview:
    """Advisor output that influences arbitration."""

    verdict: AdvisorVerdict
    reason_tags: Tuple[str, ...]
    latency_ms: float
    timestamp: float
    amended_command: Optional["ActuatorCommand"] = None
    safe_to_release: bool = False


@dataclass(frozen=True)
class SafetyCaps:
    """Speed/clearance caps injected by the safety mindset or context policies."""

    active: bool
    max_speed_mps: float
    min_clearance_m: float
    source: str
    profile: Optional[str] = None
    annotations: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class VehicleEnvelope:
    """Physical dimensions and calibration hints for the host vehicle."""

    width_m: float
    length_m: float
    height_m: float
    description: str
    clearance_margin_m: float = 0.2
    calibration_reference_distance_m: float = 2.0
    calibration_reference_pixels: float = 200.0

    def required_lateral_clearance(self) -> float:
        """Return the preferred lateral clearance on a single side."""

        return max(0.0, self.width_m / 2.0 + self.clearance_margin_m)

    def required_forward_clearance(self) -> float:
        """Return the preferred forward clearance to avoid nose collisions."""

        return max(0.0, self.length_m / 2.0 + self.clearance_margin_m)

    def meters_per_pixel_horizontal(self) -> float:
        if self.calibration_reference_pixels <= 0:
            return 0.0
        return self.width_m / self.calibration_reference_pixels

    def estimate_lane_width(self, frame_width_px: float) -> float:
        scale = self.meters_per_pixel_horizontal()
        if scale <= 0.0 or frame_width_px <= 0.0:
            return 0.0
        return frame_width_px * scale

    def estimate_distance(self, bbox: Tuple[float, float, float, float]) -> float:
        """Infer distance heuristically from a detection bounding box height."""

        _, y_min, _, y_max = bbox
        pixel_height = max(1.0, y_max - y_min)
        if self.calibration_reference_pixels <= 0 or self.calibration_reference_distance_m <= 0:
            return float("inf")
        ratio = self.calibration_reference_pixels / pixel_height
        return self.calibration_reference_distance_m * ratio

    def normalize_clearance(self, clearance_m: float) -> float:
        """Clamp a clearance value to avoid negative noise in heuristics."""

        return max(0.0, clearance_m)


@dataclass(frozen=True)
class NavigationSubGoal:
    """Represents a temporary navigation focus (e.g., merge into bike lane)."""

    goal_type: str
    status: str


@dataclass(frozen=True)
class PilotTickData:
    """Snapshot of each control tick for live dashboards/telemetry."""

    timestamp: float
    overlay: np.ndarray
    command: ActuatorCommand
    review: Optional[AdvisorReview]
    decision: NavigationDecision
    context: ContextSnapshot
    caps: SafetyCaps
    gate_tags: Tuple[str, ...]
    companion: Optional[str] = None
    lane: Optional[LaneObservation] = None



@dataclass(frozen=True)
class GPSFix:
    """Single GPS fix from an external receiver."""

    latitude: float
    longitude: float
    altitude_m: Optional[float]
    speed_mps: Optional[float]
    course_deg: Optional[float]
    timestamp: float


@dataclass(frozen=True)
class RoutePlan:
    """Geodesic plan towards a destination generated from GPS data."""

    destination_label: str
    target_latitude: float
    target_longitude: float
    distance_m: float
    bearing_deg: float
    eta_s: Optional[float] = None


@dataclass(frozen=True)
class ActuatorCommand:
    """Low level command that maps directly to throttle/steer/brake actuators."""

    steer: float    # -1.0 .. +1.0
    throttle: float  # 0.0 .. 1.0
    brake: float     # 0.0 .. 1.0


@dataclass(frozen=True)
class PerceptionSummary:
    """Aggregate information from the perception stack for visualization/logging."""

    objects: List[DetectedObject]
    frame_size: Tuple[int, int]


@dataclass(frozen=True)
class AdvisorDirective:
    """Advisor feedback used for compliance and operator feedback."""

    directive: str
    enforced_stop: bool
    caption: str
