from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Iterator, Optional, Union

import cv2


LOGGER = logging.getLogger(__name__)


class CameraSensor:
    """Polls a runtime camera image or a traditional capture device."""

    def __init__(
        self,
        source: Union[int, str, Path] = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        auto_reconnect: bool = True,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = max(1, fps)
        self.auto_reconnect = auto_reconnect
        self._capture: Optional[cv2.VideoCapture] = None
        self._poll_interval = 1.0 / float(self.fps)
        self._failure_log_interval = 1.0
        self._last_failure_log = 0.0
        self._last_failure_yield = 0.0

        if isinstance(source, Path):
            resolved = source.expanduser()
        elif isinstance(source, str) and self._looks_like_path(source):
            resolved = Path(source).expanduser()
        else:
            resolved = None

        if resolved is not None:
            self._mode = "file"
            self._image_path = resolved
            self._image_path.parent.mkdir(parents=True, exist_ok=True)
            self.source = str(resolved)
        else:
            self._mode = "device"
            if isinstance(source, str):
                try:
                    source = int(source)
                except ValueError:
                    pass
            self.source = source

    @staticmethod
    def _looks_like_path(value: str) -> bool:
        if value.startswith("~"):
            return True
        if "://" in value:
            return False
        if any(sep in value for sep in ("/", "\\")):
            return True
        lower = value.lower()
        return lower.endswith((".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"))

    def frames(self) -> Iterator[tuple[bool, Optional[cv2.Mat]]]:
        if self._mode == "file":
            yield from self._file_frames()
            return

        frame_interval = self._poll_interval
        while True:
            if self._capture is None:
                self._capture = self._open_capture()
            success, frame = self._capture.read()
            if not success:
                if not self.auto_reconnect:
                    yield success, None
                    break
                self._capture.release()
                self._capture = None
                time.sleep(0.1)
                continue
            yield success, frame
            time.sleep(frame_interval)

    def _file_frames(self) -> Iterator[tuple[bool, Optional[cv2.Mat]]]:
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

    def _open_capture(self) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(self.source)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        if not capture.isOpened():
            raise RuntimeError(f"Unable to open camera source {self.source}")
        return capture

    def _throttled_warning(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_failure_log >= self._failure_log_interval:
            self._last_failure_log = now
            LOGGER.warning(message)

    def close(self) -> None:
        if self._mode == "device" and self._capture is not None:
            self._capture.release()
            self._capture = None
