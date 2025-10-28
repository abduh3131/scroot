"""Configuration dataclasses for control arbitration."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict


@dataclass(slots=True)
class AdvisorRuntimeConfig:
    """Runtime knobs for the rule-based advisor."""

    mode: str = "normal"
    ttc_block_s: float = 1.2
    min_conf_for_allow: float = 0.55
    amend_speed_cap_mps: float = 3.0
    block_debounce_ms: int = 800
    evaluation_budget_ms: float = 4.0
    timeout_grace_ticks: int = 1


@dataclass(slots=True)
class ContextConfig:
    """Scene context interpretation parameters."""

    enabled: bool = True
    lane_bias: tuple[str, ...] = ("BIKE_LANE", "ROAD_EDGE", "SIDEWALK")
    min_confidence: float = 0.45


@dataclass(slots=True)
class NavigationIntentConfig:
    """Settings governing ambient navigation and sub-goals."""

    ambient_mode: bool = True
    prefer_safer_lane_first: bool = True
    max_lane_change_time_s: float = 2.5
    meters_stop_tolerance: float = 0.5


@dataclass(slots=True)
class SafetyMindsetProfile:
    """Profile describing conservative caps for a specific context."""

    name: str
    max_speed_mps: float
    min_clearance_m: float
    description: str


def _default_profiles() -> Dict[str, SafetyMindsetProfile]:
    return {
        "cautious_pedestrian": SafetyMindsetProfile(
            name="cautious_pedestrian",
            max_speed_mps=2.2,
            min_clearance_m=1.2,
            description="Prioritise vulnerable road users with low caps.",
        ),
        "low_visibility": SafetyMindsetProfile(
            name="low_visibility",
            max_speed_mps=1.6,
            min_clearance_m=1.5,
            description="Use when rain/fog reduces perception confidence.",
        ),
        "worst_case_child": SafetyMindsetProfile(
            name="worst_case_child",
            max_speed_mps=1.2,
            min_clearance_m=2.0,
            description="Extreme caution anticipating sudden crossings.",
        ),
    }


def _default_uncertainty_bias() -> Dict[str, float]:
    return {
        "lane_unknown_speed_cap_mps": 1.5,
        "sensor_low_conf_speed_cap_mps": 0.8,
    }


@dataclass(slots=True)
class SafetyMindsetConfig:
    """Toggleable caps applied before arbitration."""

    enabled: bool = False
    active_profile: str = "cautious_pedestrian"
    profiles: Dict[str, SafetyMindsetProfile] = field(default_factory=_default_profiles)
    uncertainty_bias: Dict[str, float] = field(default_factory=_default_uncertainty_bias)
