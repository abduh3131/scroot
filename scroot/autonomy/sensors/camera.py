from __future__ import annotations

import time
from typing import Iterator, Optional

import cv2

from scroot.utils.camera import open_camera_robust


class CameraSensor:
    """Wraps OpenCV capture with sensible defaults for autonomous operation."""

    def __init__(
        self,
        source: int | str = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        auto_reconnect: bool = True,
    ) -> None:
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps
        self.auto_reconnect = auto_reconnect
        self._capture: Optional[cv2.VideoCapture] = None

    def _open(self) -> cv2.VideoCapture:
        prefer_indices: tuple[int, ...] = ()
        prefer_paths: tuple[str, ...] = ()
        if isinstance(self.source, int):
            prefer_indices = (self.source,) + tuple(i for i in (0, 1, 2, 3) if i != self.source)
        elif isinstance(self.source, str) and self.source not in {"", "auto"}:
            prefer_paths = (self.source,) + tuple(
                p for p in ("/dev/video0", "/dev/video1", "/dev/video2") if p != self.source
            )
        capture = open_camera_robust(
            prefer_indices=prefer_indices or (0, 1, 2, 3),
            prefer_paths=prefer_paths or ("/dev/video0", "/dev/video1", "/dev/video2"),
            width=self.width,
            height=self.height,
            fps=self.fps,
        )
        return capture

    def frames(self) -> Iterator[tuple[bool, Optional[cv2.Mat]]]:
        frame_interval = 1.0 / float(self.fps)
        while True:
            if self._capture is None:
                self._capture = self._open()
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

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
