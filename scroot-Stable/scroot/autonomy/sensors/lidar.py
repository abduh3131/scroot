from __future__ import annotations

import logging
import time
import json
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from autonomy.utils.data_structures import LidarSnapshot

LOGGER = logging.getLogger(__name__)


class LidarSensor:
    """Reads LiDAR range data plus metadata from runtime SensorHub files."""

    def __init__(self, path: Path | str, meta_path: Optional[Path | str] = None) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._last_warning = 0.0
        self._warn_interval = 1.0
        if meta_path is None:
            meta_path = self.path.with_name("sensor_meta.json")
        self.meta_path = Path(meta_path).expanduser()

    def read(self) -> tuple[bool, Optional[LidarSnapshot]]:
        if not self.path.exists():
            self._throttled_warning(
                f"Waiting for LiDAR ranges at {self.path} (file not found yet)"
            )
            return False, None
        try:
            ranges = np.load(self.path, allow_pickle=False)
        except Exception as exc:  # pragma: no cover - runtime I/O issues
            self._throttled_warning(f"Failed to load LiDAR ranges: {exc}")
            return False, None
        if not isinstance(ranges, np.ndarray):
            self._throttled_warning("Loaded LiDAR data is not a NumPy array")
            return False, None
        metadata = self._load_metadata()
        snapshot = LidarSnapshot(
            ranges=np.asarray(ranges, dtype=np.float32),
            angle_min=float(metadata.get("lidar_angle_min", 0.0)),
            angle_max=float(metadata.get("lidar_angle_max", 0.0)),
            angle_increment=float(metadata.get("lidar_angle_increment", 0.0)),
            stamp=float(metadata.get("stamp", 0.0)),
            imu_vector=self._extract_imu(metadata),
            ultrasonic_distance=self._extract_ultrasonic(metadata),
        )
        return True, snapshot

    def _extract_imu(self, metadata: dict) -> Optional[Tuple[float, float, float]]:
        imu = metadata.get("imu_vector")
        if not isinstance(imu, dict):
            return None
        return (
            float(imu.get("x", 0.0)),
            float(imu.get("y", 0.0)),
            float(imu.get("z", 0.0)),
        )

    def _extract_ultrasonic(self, metadata: dict) -> Optional[float]:
        ultrasonic = metadata.get("ultrasonic_distance")
        if ultrasonic is None:
            return None
        try:
            return float(ultrasonic)
        except (TypeError, ValueError):
            return None

    def _load_metadata(self) -> dict:
        if not self.meta_path.exists():
            return {}
        try:
            data = json.loads(self.meta_path.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                return data
        except Exception as exc:  # pragma: no cover - runtime safety
            self._throttled_warning(f"Failed to parse SensorHub metadata: {exc}")
        return {}

    def _throttled_warning(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning >= self._warn_interval:
            self._last_warning = now
            LOGGER.warning(message)
