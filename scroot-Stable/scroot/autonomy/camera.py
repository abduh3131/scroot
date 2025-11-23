from __future__ import annotations

import contextlib
from dataclasses import dataclass
from typing import Generator, Optional

import cv2
import numpy as np


@dataclass
class CameraConfig:
    device: int = 0
    width: int = 640
    height: int = 480
    fps: int = 30
    pipeline: Optional[str] = None


class FrameSource:
    """USB or CSI camera reader with optional GStreamer pipeline support."""

    def __init__(self, config: CameraConfig) -> None:
        self.config = config
        self._capture = None

    def __enter__(self) -> "FrameSource":
        self._capture = self._open_capture()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def _open_capture(self) -> cv2.VideoCapture:
        if self.config.pipeline:
            cap = cv2.VideoCapture(self.config.pipeline, cv2.CAP_GSTREAMER)
        else:
            cap = cv2.VideoCapture(self.config.device)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
            cap.set(cv2.CAP_PROP_FPS, self.config.fps)
        if not cap.isOpened():
            raise RuntimeError("Unable to open camera source")
        return cap

    @contextlib.contextmanager
    def frames(self) -> Generator[np.ndarray, None, None]:
        with self:
            assert self._capture is not None
            try:
                while True:
                    ok, frame = self._capture.read()
                    if not ok or frame is None:
                        break
                    yield frame
            finally:
                if self._capture is not None:
                    self._capture.release()
                    self._capture = None
