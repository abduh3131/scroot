from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional

from autonomy.sensors.base import SensorSample


@dataclass
class LocalizationState:
    """Lightweight localization summary consumed by the summon planner."""

    timestamp: float
    quality: float
    metadata: Dict[str, float] = field(default_factory=dict)


@dataclass
class LocalizationConfig:
    lidar_range_reference: float = 8.0  # meters
    min_quality: float = 0.0
    max_quality: float = 1.0


class LocalizationModule:
    """Fuses sensor samples into a coarse localization confidence estimate."""

    def __init__(self, config: LocalizationConfig | None = None) -> None:
        self.config = config or LocalizationConfig()
        self._last_state = LocalizationState(timestamp=time.time(), quality=0.0)

    @property
    def state(self) -> LocalizationState:
        return self._last_state

    def update(self, samples: Dict[str, SensorSample]) -> LocalizationState:
        lidar_min = self._compute_lidar_min(samples.values())
        quality = self._compute_quality(lidar_min)
        state = LocalizationState(
            timestamp=time.time(),
            quality=quality,
            metadata={"lidar_min_distance": lidar_min},
        )
        self._last_state = state
        return state

    def _compute_lidar_min(self, samples: Iterable[SensorSample]) -> Optional[float]:
        min_distance: Optional[float] = None
        for sample in samples:
            data = sample.data
            if isinstance(data, dict) and "ranges" in data:
                ranges = [float(r) for r in data.get("ranges", []) if r is not None and r > 0]
                if not ranges:
                    continue
                candidate = min(ranges)
                if min_distance is None or candidate < min_distance:
                    min_distance = candidate
        return min_distance

    def _compute_quality(self, lidar_min: Optional[float]) -> float:
        if lidar_min is None:
            return self.config.min_quality
        reference = max(self.config.lidar_range_reference, 1e-3)
        normalized = max(0.0, min(lidar_min / reference, 1.0))
        quality = self.config.max_quality * (1.0 - normalized)
        return max(self.config.min_quality, min(self.config.max_quality, quality))
