from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterator, Optional

import time


@dataclass
class SensorSample:
    """Represents a single reading emitted by a sensor."""

    name: str
    timestamp: float
    data: Any
    ok: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)


class Sensor(ABC):
    """Abstract base class for all plug-and-play sensors."""

    name: str

    def __init__(self, name: str) -> None:
        self.name = name

    @abstractmethod
    def start(self) -> None:
        """Initialize the sensor and prepare for data capture."""

    @abstractmethod
    def stop(self) -> None:
        """Release any resources used by the sensor."""

    @abstractmethod
    def read(self) -> SensorSample:
        """Return the latest measurement from the sensor."""


class StreamingSensor(Sensor):
    """Base class for sensors that stream frames or samples."""

    @abstractmethod
    def stream(self) -> Iterator[SensorSample]:
        """Yield measurements continuously until stopped."""

    def _make_sample(self, data: Any, ok: bool = True, metadata: Optional[Dict[str, Any]] = None) -> SensorSample:
        """Helper to construct samples with consistent timestamps."""
        return SensorSample(
            name=self.name,
            timestamp=time.time(),
            data=data,
            ok=ok,
            metadata=metadata or {},
        )


@dataclass
class SensorFactory:
    """Registry entry describing how to construct a sensor."""

    constructor: Callable[..., Sensor]
    description: str


class SensorRegistry:
    """Global registry tracking sensor adapters by key."""

    def __init__(self) -> None:
        self._registry: Dict[str, SensorFactory] = {}

    def register(self, key: str, constructor: Callable[..., Sensor], description: str) -> None:
        if key in self._registry:
            raise ValueError(f"Sensor '{key}' is already registered")
        self._registry[key] = SensorFactory(constructor=constructor, description=description)

    def create(self, key: str, *args: Any, **kwargs: Any) -> Sensor:
        if key not in self._registry:
            available = ", ".join(sorted(self._registry))
            raise KeyError(f"Unknown sensor '{key}'. Available adapters: {available}")
        factory = self._registry[key]
        return factory.constructor(*args, **kwargs)

    def describe(self, key: str) -> Optional[str]:
        factory = self._registry.get(key)
        return factory.description if factory else None

    def keys(self) -> Iterator[str]:
        return iter(self._registry.keys())


global_sensor_registry = SensorRegistry()
