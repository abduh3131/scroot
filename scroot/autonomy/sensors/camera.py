from __future__ import annotations

import glob
import logging
import pathlib
import time
from typing import Iterator, Optional, Sequence, Tuple, Union

import cv2

from .base import SensorSample, StreamingSensor, global_sensor_registry

LOG = logging.getLogger(__name__)

GSTREAMER_PREFIXES = ("nvarguscamerasrc", "v4l2src")
DEFAULT_MAX_PROBE = 8


class CameraSensor(StreamingSensor):
    """OpenCV-backed video capture sensor with plug-and-play registration."""

    def __init__(
        self,
        source: int | str = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        auto_reconnect: bool = True,
        probe_sources: Optional[Sequence[Union[int, str]]] = None,
        prefer_gstreamer: Optional[bool] = None,
        gstreamer_flip_method: int = 0,
        max_probe_sources: int = DEFAULT_MAX_PROBE,
        name: Optional[str] = None,
    ) -> None:
        super().__init__(name or f"camera:{source}")
        self.source = self._normalize_source(source)
        self.width = width
        self.height = height
        self.fps = fps
        self.auto_reconnect = auto_reconnect
        self._prefer_gstreamer = prefer_gstreamer
        self._gstreamer_flip_method = gstreamer_flip_method
        self._max_probe_sources = max(1, int(max_probe_sources))
        self._capture: Optional[cv2.VideoCapture] = None
        self._frame_interval = 1.0 / float(self.fps)
        self._active_source: Optional[Union[int, str]] = None
        self._probe_sources = [self._normalize_source(value) for value in (probe_sources or [])]

    def start(self) -> None:
        if self._capture is None:
            self._capture = self._open()
            LOG.info("Camera '%s' opened using source %s", self.name, self._active_source)

    def stop(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
            LOG.info("Camera '%s' released", self.name)

    def read(self) -> SensorSample:
        if self._capture is None:
            self.start()
        assert self._capture is not None
        success, frame = self._capture.read()
        if success and frame is None:
            success = False
        if not success and self.auto_reconnect:
            LOG.warning("Camera '%s' read failed; attempting reconnect", self.name)
            self.stop()
            time.sleep(0.1)
            self.start()
            assert self._capture is not None
            success, frame = self._capture.read()
            if success and frame is None:
                success = False
        metadata = {
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "source": self._active_source,
        }
        return self._make_sample(data=frame, ok=success, metadata=metadata)

    def stream(self) -> Iterator[SensorSample]:
        while True:
            sample = self.read()
            if not sample.ok and not self.auto_reconnect:
                yield sample
                break
            yield sample
            time.sleep(self._frame_interval)

    def frames(self) -> Iterator[Tuple[bool, Optional[cv2.Mat]]]:
        """Backward-compatible generator name retained for existing callers."""
        for sample in self.stream():
            yield sample.ok, sample.data

    def close(self) -> None:
        self.stop()

    def _open(self) -> cv2.VideoCapture:
        candidates = self._candidate_sources()
        last_error: Optional[str] = None
        for candidate in candidates:
            capture = self._try_open(candidate)
            if capture is None:
                continue
            if capture.isOpened():
                self._active_source = candidate
                self._configure_capture(capture, candidate)
                return capture
            last_error = f"VideoCapture opened but invalid for {candidate}"
            capture.release()

        summary = ", ".join(str(value) for value in candidates) if candidates else "<none>"
        message = f"Unable to open camera. Tried: {summary}"
        if last_error:
            message = f"{message} (last error: {last_error})"
        raise RuntimeError(message)

    def _candidate_sources(self) -> list[Union[int, str]]:
        candidates: list[Union[int, str]] = []

        def add_unique(values: Sequence[Union[int, str]]) -> None:
            for value in values:
                if value in candidates:
                    continue
                candidates.append(value)

        for value in self._probe_sources:
            add_unique(self._expand_source_value(value))

        add_unique(self._expand_source_value(self.source))

        if self._should_auto_discover(self.source):
            for index in range(self._max_probe_sources):
                add_unique(self._expand_source_value(index))
            for device in sorted(glob.glob("/dev/video*")):
                add_unique(self._expand_source_value(device))
            if self._should_probe_gstreamer():
                for sensor_id in range(self._max_probe_sources):
                    add_unique([self._build_gstreamer_pipeline(sensor_id)])

        return candidates

    def _expand_source_value(self, value: Union[int, str]) -> list[Union[int, str]]:
        if isinstance(value, int):
            return [value]
        if value == "auto":
            return []
        if self._is_csi_reference(value):
            sensor_id = self._parse_csi_reference(value)
            return [self._build_gstreamer_pipeline(sensor_id)]
        if value.startswith("gstreamer://"):
            return [value.split("gstreamer://", 1)[1]]
        if value.startswith("gstreamer:"):
            return [value.split("gstreamer:", 1)[1]]
        return [value]

    def _try_open(self, candidate: Union[int, str]) -> Optional[cv2.VideoCapture]:
        try:
            if isinstance(candidate, str) and self._looks_like_gstreamer(candidate):
                capture = cv2.VideoCapture(candidate, cv2.CAP_GSTREAMER)
            else:
                capture = cv2.VideoCapture(candidate)
        except Exception as exc:  # pragma: no cover - defensive logging
            LOG.error("Camera '%s' failed to open %s: %s", self.name, candidate, exc)
            return None

        if not capture.isOpened():
            LOG.debug("Camera '%s' candidate %s did not open", self.name, candidate)
            capture.release()
            return None
        return capture

    def _configure_capture(self, capture: cv2.VideoCapture, candidate: Union[int, str]) -> None:
        if isinstance(candidate, str) and self._looks_like_gstreamer(candidate):
            return
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)

    def _normalize_source(self, value: Union[int, str]) -> Union[int, str]:
        if isinstance(value, int):
            return value
        text = str(value).strip()
        if text.lower() in {"auto", "detect"}:
            return "auto"
        try:
            return int(text)
        except ValueError:
            return text

    def _should_auto_discover(self, value: Union[int, str]) -> bool:
        return isinstance(value, str) and value == "auto"

    def _looks_like_gstreamer(self, value: str) -> bool:
        stripped = value.strip()
        return any(stripped.startswith(prefix) for prefix in GSTREAMER_PREFIXES) or "!" in stripped

    def _should_probe_gstreamer(self) -> bool:
        if self._prefer_gstreamer is not None:
            return self._prefer_gstreamer
        return _is_jetson()

    def _is_csi_reference(self, value: str) -> bool:
        lowered = value.lower()
        return lowered.startswith("csi://") or lowered.startswith("csi:")

    def _parse_csi_reference(self, value: str) -> int:
        if "://" in value:
            suffix = value.split("://", 1)[1]
        elif ":" in value:
            suffix = value.split(":", 1)[1]
        else:
            suffix = value
        try:
            return max(int(suffix), 0)
        except ValueError:
            return 0

    def _build_gstreamer_pipeline(self, sensor_id: int) -> str:
        fps_int = max(int(round(self.fps)), 1)
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={self.width}, height={self.height}, framerate={fps_int}/1 ! "
            f"nvvidconv flip-method={self._gstreamer_flip_method} ! "
            f"video/x-raw, width={self.width}, height={self.height}, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink"
        )


def _is_jetson() -> bool:
    model_path = pathlib.Path("/proc/device-tree/model")
    try:
        text = model_path.read_text(encoding="utf-8")
    except OSError:
        return False
    return "nvidia" in text.lower()


global_sensor_registry.register(
    "camera",
    lambda **kwargs: CameraSensor(**kwargs),
    "OpenCV compatible video capture device",
)
