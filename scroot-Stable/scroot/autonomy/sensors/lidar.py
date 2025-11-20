from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Optional

import numpy as np

LOGGER = logging.getLogger(__name__)


class LidarSensor:
    """Reads LiDAR range data from a runtime `.npy` file."""

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._last_warning = 0.0
        self._warn_interval = 1.0

    def read(self) -> tuple[bool, Optional[np.ndarray]]:
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
        return True, np.asarray(ranges, dtype=np.float32)

    def _throttled_warning(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning >= self._warn_interval:
            self._last_warning = now
            LOGGER.warning(message)
