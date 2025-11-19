from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Iterator, Optional, Union

import cv2


LOGGER = logging.getLogger(__name__)


class CameraSensor:
    """Polls a runtime camera image exported by the ROS bridge."""

    def __init__(
        self,
        source: Union[int, str, Path] = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        auto_reconnect: bool = True,
    ) -> None:
        del auto_reconnect  # maintained for compatibility with old signatures
        self.width = width
        self.height = height
        self.fps = max(1, fps)
        self._poll_interval = 1.0 / float(self.fps)
        self._failure_log_interval = 1.0
        self._last_failure_log = 0.0
        self._last_failure_yield = 0.0

        if isinstance(source, (int, float)):
            raise ValueError(
                "CameraSensor no longer opens hardware devices directly. "
                "Point it at the runtime snapshot written by ai_input_bridge."
            )

        path = Path(str(source)).expanduser()
        self._image_path = path
        self._image_path.parent.mkdir(parents=True, exist_ok=True)
        self.source = str(path)

    def frames(self) -> Iterator[tuple[bool, Optional[cv2.Mat]]]:
        while True:
            frame = None
            if self._image_path.exists():
                frame = cv2.imread(str(self._image_path))
            if frame is None:
                self._throttled_warning(
                    f"Waiting for camera file at {self._image_path} to be populated"
                )
                if self._should_emit_failure():
                    yield False, None
                time.sleep(self._poll_interval)
                continue
            yield True, frame
            time.sleep(self._poll_interval)

    def _should_emit_failure(self) -> bool:
        now = time.monotonic()
        if now - self._last_failure_yield >= self._failure_log_interval:
            self._last_failure_yield = now
            return True
        return False

    def _throttled_warning(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_failure_log >= self._failure_log_interval:
            self._last_failure_log = now
            LOGGER.warning(message)

    def close(self) -> None:
        # No hardware handles to release when mirroring files.
        return
