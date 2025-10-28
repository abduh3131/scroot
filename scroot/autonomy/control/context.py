"""Scene context evaluation utilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from autonomy.control.config import ContextConfig, NavigationIntentConfig
from autonomy.utils.data_structures import ContextSnapshot, LaneType, NavigationDecision


@dataclass(slots=True)
class SceneContext:
    snapshot: ContextSnapshot
    subgoal_hint: Optional[str]


class ContextAnalyzer:
    """Derives semantic lane and confidence cues for the advisor."""

    def __init__(self, config: ContextConfig, intent_config: NavigationIntentConfig) -> None:
        self.config = config
        self.intent_config = intent_config

    def analyze(self, decision: NavigationDecision) -> SceneContext:
        metadata = dict(decision.metadata)
        lane_str = metadata.get("lane_type", "UNKNOWN").upper()
        lane_type = LaneType[lane_str] if lane_str in LaneType.__members__ else LaneType.UNKNOWN

        confidence = float(metadata.get("lane_confidence", 0.0))
        sensor_confidence = float(metadata.get("sensor_confidence", metadata.get("perception_confidence", 1.0)))
        recommended_bias = float(metadata.get("recommended_bias", decision.steering_bias))

        if not self.config.enabled:
            lane_type = LaneType.UNKNOWN
            confidence = 0.0

        safer_hint: Optional[str] = None
        if self.intent_config.prefer_safer_lane_first:
            safer_hint = metadata.get("safer_lane_available")
            if not safer_hint:
                bike_lane_offset = metadata.get("bike_lane_offset", 0.0)
                if bike_lane_offset:
                    safer_hint = "right" if bike_lane_offset > 0 else "left"

        snapshot = ContextSnapshot(
            lane_type=lane_type,
            confidence=confidence,
            sensor_confidence=max(0.0, min(1.0, sensor_confidence)),
            recommended_bias=max(-1.0, min(1.0, recommended_bias)),
            metadata=metadata,
        )
        return SceneContext(snapshot=snapshot, subgoal_hint=safer_hint)
