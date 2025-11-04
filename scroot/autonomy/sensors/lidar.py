from __future__ import annotations

import io
import json
import math
import pathlib
import time
from typing import Iterator, List, Optional

from .base import SensorSample, StreamingSensor, global_sensor_registry


class LiDARSensor(StreamingSensor):
    """Simple LiDAR adapter that replays scans from JSONL/CSV logs or synthetic data."""

    def __init__(
        self,
        source: Optional[str] = None,
        loop: bool = True,
        frame_rate: float = 10.0,
        name: Optional[str] = None,
    ) -> None:
        super().__init__(name or "lidar")
        self.source = source
        self.loop = loop
        self.frame_interval = 1.0 / max(frame_rate, 1e-3)
        self._stream: Optional[io.TextIOBase] = None

    def start(self) -> None:
        if self.source is None:
            return
        path = pathlib.Path(self.source).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"LiDAR source '{self.source}' not found")
        self._stream = path.open("r", encoding="utf-8")

    def stop(self) -> None:
        if self._stream:
            self._stream.close()
            self._stream = None

    def read(self) -> SensorSample:
        data = self._next_scan()
        return self._make_sample(data=data, ok=data is not None)

    def stream(self) -> Iterator[SensorSample]:
        while True:
            sample = self.read()
            if not sample.ok:
                if not self.loop:
                    yield sample
                    break
                self._restart_stream()
                continue
            yield sample
            time.sleep(self.frame_interval)

    def _next_scan(self) -> Optional[dict]:
        if self._stream is None:
            # Generate a synthetic 2D ring if no source provided.
            ranges = [3.0 + math.sin(idx / 10.0) for idx in range(360)]
            return {
                "ranges": ranges,
                "angle_min": 0.0,
                "angle_increment": math.radians(1.0),
            }

        line = self._stream.readline()
        if not line:
            return None

        line = line.strip()
        if not line:
            return self._next_scan()

        if line.startswith("{"):
            payload = json.loads(line)
            return payload

        # Assume CSV: distance values per line
        ranges = [float(value) for value in line.split(",") if value.strip()]
        return {
            "ranges": ranges,
            "angle_min": 0.0,
            "angle_increment": (2.0 * math.pi) / max(len(ranges), 1),
        }

    def _restart_stream(self) -> None:
        self.stop()
        self.start()


global_sensor_registry.register(
    "lidar",
    lambda **kwargs: LiDARSensor(**kwargs),
    "Replay or synthetic LiDAR scan provider",
)
