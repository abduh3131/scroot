from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from autonomy.utils.data_structures import NavigationDecision, VehicleMode
from autonomy.planning.localization import LocalizationState


@dataclass
class SummonPlannerConfig:
    low_quality_threshold: float = 0.35
    low_quality_speed_scale: float = 0.4
    nominal_goal_context: str = "summon"


class SummonPlanner:
    """Adjusts navigation decisions when operating in summon mode."""

    def __init__(self, config: SummonPlannerConfig | None = None) -> None:
        self.config = config or SummonPlannerConfig()

    def adjust(
        self,
        decision: NavigationDecision,
        mode: VehicleMode,
        localization: LocalizationState,
    ) -> NavigationDecision:
        if mode != VehicleMode.SUMMON:
            return decision

        desired_speed = decision.desired_speed
        goal_context = decision.goal_context or self.config.nominal_goal_context

        if localization.quality < self.config.low_quality_threshold:
            desired_speed *= self.config.low_quality_speed_scale

        return NavigationDecision(
            steering_bias=decision.steering_bias,
            desired_speed=desired_speed,
            hazard_level=decision.hazard_level,
            metadata=dict(decision.metadata),
            enforced_stop=decision.enforced_stop,
            directive=decision.directive,
            caption=decision.caption,
            goal_context=goal_context,
            mode=decision.mode,
        )
