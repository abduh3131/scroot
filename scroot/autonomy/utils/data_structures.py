from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


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
class NavigationSubGoal:
    """Represents a temporary navigation focus (e.g., merge into bike lane)."""

    goal_type: str
    status: str


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
