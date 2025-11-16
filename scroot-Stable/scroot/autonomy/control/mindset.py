"""Safety mindset implementation that emits caps for arbitration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from autonomy.control.config import SafetyMindsetConfig
from autonomy.utils.data_structures import (
    ContextSnapshot,
    NavigationDecision,
    SafetyCaps,
    VehicleEnvelope,
)


@dataclass
class SafetyMindsetState:
    active_profile: Optional[str] = None


class SafetyMindset:
    """Applies configured caps based on the chosen mindset and uncertainty."""

    def __init__(self, config: SafetyMindsetConfig, vehicle: Optional[VehicleEnvelope] = None) -> None:
        self.config = config
        self.state = SafetyMindsetState(active_profile=config.active_profile if config.enabled else None)
        self.vehicle = vehicle

    def evaluate(self, context: ContextSnapshot, decision: NavigationDecision) -> SafetyCaps:
        if not self.config.enabled:
            return SafetyCaps(
                active=False,
                max_speed_mps=decision.desired_speed,
                min_clearance_m=self.vehicle.required_forward_clearance() if self.vehicle else 0.5,
                source="default",
                annotations={"mindset_enabled": 0.0},
            )

        profile_key = self.state.active_profile or self.config.active_profile
        profile = self.config.profiles.get(profile_key)
        if profile is None:
            profile = next(iter(self.config.profiles.values()))
            profile_key = profile.name
            self.state.active_profile = profile_key

        max_speed = profile.max_speed_mps
        min_clearance = profile.min_clearance_m
        annotations = {"profile": profile_key}

        if self.vehicle:
            min_clearance = max(min_clearance, self.vehicle.required_forward_clearance())
            annotations["vehicle_forward_clearance"] = self.vehicle.required_forward_clearance()

        if context.confidence < 0.5:
            cap = self.config.uncertainty_bias.get("lane_unknown_speed_cap_mps")
            if cap is not None:
                max_speed = min(max_speed, cap)
                annotations["lane_unknown_cap"] = cap

        if context.sensor_confidence < 0.6:
            cap = self.config.uncertainty_bias.get("sensor_low_conf_speed_cap_mps")
            if cap is not None:
                max_speed = min(max_speed, cap)
                annotations["sensor_low_conf"] = context.sensor_confidence

        return SafetyCaps(
            active=True,
            max_speed_mps=max_speed,
            min_clearance_m=min_clearance,
            source="mindset",
            profile=profile_key,
            annotations=annotations,
        )
