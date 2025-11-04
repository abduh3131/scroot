from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, Iterator, List, Optional

from autonomy.sensors.base import SensorSample, StreamingSensor, global_sensor_registry


@dataclass
class SensorSpec:
    """Declarative description of a sensor adapter and its constructor args."""

    adapter: str
    name: Optional[str] = None
    args: Dict[str, object] = field(default_factory=dict)


class SensorManager:
    """Handles lifecycle for a heterogeneous collection of streaming sensors."""

    def __init__(self, specs: Iterable[SensorSpec], primary_name: Optional[str] = None) -> None:
        specs = list(specs)
        if not specs:
            raise ValueError("SensorManager requires at least one sensor specification")

        self._sensors: Dict[str, StreamingSensor] = {}
        for spec in specs:
            name = spec.name or spec.adapter
            if name in self._sensors:
                raise ValueError(f"Duplicate sensor name '{name}' in configuration")
            sensor = global_sensor_registry.create(spec.adapter, name=name, **spec.args)
            if not isinstance(sensor, StreamingSensor):
                raise TypeError(f"Sensor '{name}' must inherit from StreamingSensor")
            self._sensors[name] = sensor

        self._primary_name = primary_name or next(iter(self._sensors))
        if self._primary_name not in self._sensors:
            raise ValueError(f"Primary sensor '{self._primary_name}' not found in configuration")

    @property
    def names(self) -> List[str]:
        return list(self._sensors.keys())

    @property
    def primary(self) -> StreamingSensor:
        return self._sensors[self._primary_name]

    def get(self, name: str) -> StreamingSensor:
        return self._sensors[name]

    def start_all(self) -> None:
        for sensor in self._sensors.values():
            sensor.start()

    def stop_all(self) -> None:
        for sensor in self._sensors.values():
            sensor.stop()

    def stream(self, name: Optional[str] = None) -> Iterator[SensorSample]:
        sensor = self.primary if name is None else self.get(name)
        yield from sensor.stream()

    def samples(self) -> Dict[str, SensorSample]:
        """Grab a single sample from each configured sensor."""
        snapshots: Dict[str, SensorSample] = {}
        for name, sensor in self._sensors.items():
            snapshots[name] = sensor.read()
        return snapshots
