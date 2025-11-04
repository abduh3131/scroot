from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple

from autonomy.sensors.base import SensorSample
from autonomy.utils.data_structures import NavigationDecision, PerceptionSummary, VehicleMode


@dataclass
class GuardianConfig:
    """Parameters controlling the proactive safety layer."""

    slowdown_hazard_threshold: float = 0.5
    emergency_hazard_threshold: float = 0.85
    minimum_speed_scale: float = 0.2
    priority_labels: Tuple[str, ...] = ("person", "bicycle", "traffic light", "stop sign")
    alert_on_priority: bool = True
    lidar_slowdown_distance: float = 2.5
    lidar_stop_distance: float = 1.0


@dataclass
class GuardianDecision:
    """Output describing safety constraints for the current timestep."""

    force_stop: bool
    speed_scale: float
    alerts: List[str] = field(default_factory=list)


class Guardian:
    """Watches perception + navigation outputs and enforces conservative overrides."""

    def __init__(self, config: GuardianConfig | None = None) -> None:
        self.config = config or GuardianConfig()

    def evaluate(
        self,
        perception: PerceptionSummary,
        decision: NavigationDecision,
        mode: VehicleMode,
        lidar_samples: Optional[List[SensorSample]] = None,
    ) -> GuardianDecision:
        alerts: List[str] = []
        force_stop = decision.enforced_stop
        speed_scale = 1.0

        hazard = decision.hazard_level
        if hazard >= self.config.emergency_hazard_threshold:
            force_stop = True
            alerts.append("hazard_emergency")
        elif hazard >= self.config.slowdown_hazard_threshold:
            scale = max(self.config.minimum_speed_scale, 1.0 - hazard)
            speed_scale = min(speed_scale, scale)
            alerts.append("hazard_slowdown")

        if self.config.alert_on_priority:
            if self._priority_object_detected(perception.objects):
                alerts.append("priority_object")

        if lidar_samples:
            lidar_alerts, lidar_scale, lidar_force_stop = self._evaluate_lidar(lidar_samples)
            alerts.extend(lidar_alerts)
            speed_scale = min(speed_scale, lidar_scale)
            force_stop = force_stop or lidar_force_stop

        return GuardianDecision(
            force_stop=force_stop,
            speed_scale=speed_scale,
            alerts=alerts,
        )

    def _priority_object_detected(self, detections: Iterable) -> bool:
        priority = {label.lower() for label in self.config.priority_labels}
        for detection in detections:
            label = getattr(detection, "label", None)
            if label and label.lower() in priority:
                return True
        return False

    def _evaluate_lidar(self, samples: List[SensorSample]) -> Tuple[List[str], float, bool]:
        min_distance: Optional[float] = None
        for sample in samples:
            payload = sample.data
            if isinstance(payload, dict) and "ranges" in payload:
                ranges = [r for r in payload.get("ranges", []) if isinstance(r, (int, float)) and r > 0]
                if not ranges:
                    continue
                candidate = min(ranges)
                if min_distance is None or candidate < min_distance:
                    min_distance = candidate
        if min_distance is None:
            return [], 1.0, False

        alerts: List[str] = []
        force_stop = False
        speed_scale = 1.0

        if min_distance <= self.config.lidar_stop_distance:
            alerts.append("lidar_stop")
            force_stop = True
        elif min_distance <= self.config.lidar_slowdown_distance:
            alerts.append("lidar_slowdown")
            ratio = max(min_distance / max(self.config.lidar_slowdown_distance, 1e-3), self.config.minimum_speed_scale)
            speed_scale = min(speed_scale, ratio)

        return alerts, speed_scale, force_stop
