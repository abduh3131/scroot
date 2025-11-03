"""Automatic USB camera discovery and recovery for the scooter app.

This module encapsulates a fault-tolerant camera pipeline that discovers
available `/dev/video*` nodes, probes them for usable formats, and feeds a
single, continuously updating frame buffer back to the rest of the
application. It prefers MJPEG modes that behave well with USB/IP and WSL2,
falls back to raw YUYV when needed, and automatically retries other modes
or devices whenever the active capture stalls.

Usage
-----

```
from camera_autopilot import open as open_camera

handle = open_camera()
handle.start()
frame = handle.get_frame()
...
handle.stop()
```

You can also run ``python -m camera_autopilot --preview`` to see the
auto-detected device/mode and live FPS for a few seconds.
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

import cv2
import numpy as np


LOGGER = logging.getLogger("camera_autopilot")

SAFE_PROBE_MODES: Sequence[Tuple[str, int, int, int]] = (
    ("MJPG", 640, 480, 30),
    ("MJPG", 320, 240, 15),
    ("YUYV", 640, 480, 30),
    ("YUYV", 320, 240, 15),
)

STALL_TIMEOUT_SECONDS = 1.5
CAPTURE_BACKEND = getattr(cv2, "CAP_V4L2", 200)
ERROR_MESSAGE = (
    "No webcam delivered frames. Try reattaching device (usbipd), or a MJPEG-capable camera."
)


def _default_nodes() -> List[str]:
    return [f"/dev/video{i}" for i in range(10)]


@dataclass(frozen=True)
class CameraMode:
    fourcc: str
    width: int
    height: int
    fps: int

    def descriptor(self) -> str:
        return f"{self.fourcc} {self.width}x{self.height}@{self.fps}"


@dataclass(frozen=True)
class CameraSelection:
    node: str
    mode: CameraMode
    init_ms: int
    label: Optional[str] = None


def _ensure_video_group(logger: logging.Logger) -> None:
    if os.name != "posix":
        return
    if hasattr(os, "getuid") and os.getuid() == 0:
        return
    try:
        import getpass
        import grp
    except Exception:
        return
    try:
        video_group = grp.getgrnam("video")
    except KeyError:
        return
    username = getpass.getuser()
    user_groups = os.getgroups()
    if video_group.gr_gid not in user_groups:
        logger.warning(
            "User %s is not in the 'video' group; USB cameras may refuse access.",
            username,
        )


def _normalize_nodes(nodes: Optional[Sequence[str | int]]) -> List[str]:
    if not nodes:
        return []
    normalized: List[str] = []
    for entry in nodes:
        if isinstance(entry, int):
            normalized.append(f"/dev/video{entry}")
        else:
            text = str(entry)
            if not text.startswith("/dev/video") and text.isdigit():
                normalized.append(f"/dev/video{text}")
            elif not text.startswith("/dev/video") and text.startswith("video"):
                normalized.append(f"/dev/{text}")
            else:
                normalized.append(text)
    return normalized


def _discover_nodes(preferred: Optional[Sequence[str | int]] = None) -> List[str]:
    preferred_nodes = _normalize_nodes(preferred)
    candidates = _default_nodes()
    nodes: List[str] = []
    for node in candidates:
        if os.path.exists(node) and os.access(node, os.R_OK):
            nodes.append(node)
    ordered: List[str] = []
    for node in preferred_nodes:
        if node in nodes and node not in ordered:
            ordered.append(node)
    for node in nodes:
        if node not in ordered:
            ordered.append(node)
    return ordered


def _query_device_labels() -> Dict[str, str]:
    labels: Dict[str, str] = {}
    v4l2ctl = shutil.which("v4l2-ctl")
    if not v4l2ctl:
        return labels
    try:
        proc = subprocess.run(
            [v4l2ctl, "--list-devices"],
            capture_output=True,
            text=True,
            check=False,
        )
    except Exception:
        return labels
    current_label: Optional[str] = None
    for line in proc.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if not stripped.startswith("/dev/video"):
            current_label = stripped.rstrip(":")
            continue
        if current_label:
            labels[stripped] = current_label
    return labels


def _list_modes_v4l2(node: str) -> List[CameraMode]:
    v4l2ctl = shutil.which("v4l2-ctl")
    if not v4l2ctl:
        return []
    try:
        proc = subprocess.run(
            [v4l2ctl, f"--list-formats-ext={node}"],
            capture_output=True,
            text=True,
            check=False,
        )
    except Exception:
        return []
    modes: List[CameraMode] = []
    current_fourcc: Optional[str] = None
    current_size: Optional[Tuple[int, int]] = None
    size_pattern = re.compile(r"Size: Discrete (\d+)x(\d+)")
    interval_pattern = re.compile(r"Interval: Discrete [^()]*\(([^)]+) fps\)")
    for line in proc.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("[") and "'" in stripped:
            match = re.search(r"'([A-Z0-9]{4})'", stripped)
            if match:
                current_fourcc = match.group(1)
            continue
        size_match = size_pattern.search(stripped)
        if size_match:
            current_size = (int(size_match.group(1)), int(size_match.group(2)))
            continue
        interval_match = interval_pattern.search(stripped)
        if interval_match and current_fourcc and current_size:
            try:
                fps_value = float(interval_match.group(1))
            except ValueError:
                continue
            if fps_value <= 0:
                continue
            fps = int(round(fps_value))
            mode = CameraMode(
                fourcc=current_fourcc,
                width=current_size[0],
                height=current_size[1],
                fps=max(1, fps),
            )
            if mode not in modes:
                modes.append(mode)
    return modes


def _video_writer_fourcc(fourcc: str) -> int:
    fourcc = (fourcc or "MJPG").upper()
    return cv2.VideoWriter_fourcc(*fourcc)


def _set_if_supported(capture: cv2.VideoCapture, prop: int, value: float) -> None:
    try:
        capture.set(prop, value)
    except Exception:
        pass


class _ProbeResult:
    def __init__(self, success: bool, init_ms: int = 0) -> None:
        self.success = success
        self.init_ms = init_ms


class _CameraAutopilotCore:
    def __init__(
        self,
        preferred_nodes: Optional[Sequence[str | int]] = None,
        preferred_mode: Optional[CameraMode] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.logger = logger or LOGGER
        if "OPENCV_LOG_LEVEL" not in os.environ:
            os.environ["OPENCV_LOG_LEVEL"] = "ERROR"
        env_node = os.environ.get("SCROOT_CAMERA_NODE")
        env_mode = os.environ.get("SCROOT_CAMERA_MODE")
        node_overrides: List[str] = []
        if env_node:
            node_overrides = _normalize_nodes([env_node])
        mode_override = preferred_mode
        if env_mode:
            parsed = self._parse_mode(env_mode)
            if parsed:
                mode_override = parsed
        self.preferred_mode = mode_override
        nodes = node_overrides or _normalize_nodes(preferred_nodes)
        self.preferred_nodes = nodes
        self.labels = _query_device_labels()
        _ensure_video_group(self.logger)

    @staticmethod
    def _parse_mode(token: str) -> Optional[CameraMode]:
        text = token.strip().upper()
        match = re.match(r"([A-Z0-9]{4})[:\s]*(\d+)X(\d+)@?(\d+)?", text)
        if not match:
            return None
        fourcc = match.group(1)
        width = int(match.group(2))
        height = int(match.group(3))
        fps = int(match.group(4) or 30)
        return CameraMode(fourcc=fourcc, width=width, height=height, fps=fps)

    def initial_selection(self) -> CameraSelection:
        nodes = _discover_nodes(self.preferred_nodes)
        if not nodes:
            raise RuntimeError(ERROR_MESSAGE)
        selection = self._sweep(nodes)
        label = self.labels.get(selection.node)
        if label:
            selection = dataclasses.replace(selection, label=label)
        self.logger.info(
            "Selected camera %s (%s) in %.1f ms",
            selection.node,
            selection.mode.descriptor(),
            selection.init_ms,
        )
        return selection

    def recover(self, last: Optional[CameraSelection], reason: str) -> CameraSelection:
        nodes = _discover_nodes(self.preferred_nodes)
        if not nodes:
            raise RuntimeError(ERROR_MESSAGE)
        ordered: List[str] = []
        if last:
            ordered.append(last.node)
        for node in nodes:
            if node not in ordered:
                ordered.append(node)
        selection = self._sweep(ordered)
        label = self.labels.get(selection.node)
        if label:
            selection = dataclasses.replace(selection, label=label)
        self.logger.info(
            "Camera recovery: %s (%s) after %s",
            selection.node,
            selection.mode.descriptor(),
            reason,
        )
        return selection

    def _sweep(self, nodes: Sequence[str]) -> CameraSelection:
        plan_cache: Dict[str, List[CameraMode]] = {}
        for sweep in range(2):
            for node in nodes:
                plan = plan_cache.get(node)
                if plan is None:
                    plan = self._build_probe_plan(node)
                    if not plan:
                        continue
                    plan_cache[node] = plan
                for mode in plan:
                    result = self._probe(node, mode)
                    if result.success:
                        return CameraSelection(node=node, mode=mode, init_ms=result.init_ms)
            time.sleep(1.0)
        raise RuntimeError(ERROR_MESSAGE)

    def _build_probe_plan(self, node: str) -> List[CameraMode]:
        plan: List[CameraMode] = []
        seen: set[Tuple[str, int, int, int]] = set()

        def push(mode: CameraMode) -> None:
            key = (mode.fourcc, mode.width, mode.height, mode.fps)
            if key not in seen:
                plan.append(mode)
                seen.add(key)

        if self.preferred_mode:
            push(self.preferred_mode)

        for spec in SAFE_PROBE_MODES:
            push(CameraMode(*spec))

        extra_modes = _list_modes_v4l2(node)
        mjpg_extras = [m for m in extra_modes if m.fourcc.upper() == "MJPG" and m not in plan]
        for mode in mjpg_extras:
            push(mode)
        for mode in extra_modes:
            if mode.fourcc.upper() == "MJPG":
                continue
            push(mode)
        return plan

    def _probe(self, node: str, mode: CameraMode) -> _ProbeResult:
        start = time.perf_counter()
        capture = cv2.VideoCapture(node, CAPTURE_BACKEND)
        if not capture or not capture.isOpened():
            return _ProbeResult(False)
        try:
            _set_if_supported(capture, cv2.CAP_PROP_BUFFERSIZE, 1)
            _set_if_supported(capture, cv2.CAP_PROP_FOURCC, _video_writer_fourcc(mode.fourcc))
            _set_if_supported(capture, cv2.CAP_PROP_FRAME_WIDTH, mode.width)
            _set_if_supported(capture, cv2.CAP_PROP_FRAME_HEIGHT, mode.height)
            _set_if_supported(capture, cv2.CAP_PROP_FPS, mode.fps)
            open_timeout = getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", None)
            read_timeout = getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", None)
            if open_timeout is not None:
                _set_if_supported(capture, open_timeout, 1000)
            if read_timeout is not None:
                _set_if_supported(capture, read_timeout, 1000)

            success_count = 0
            attempts = 0
            while attempts < 60 and (time.perf_counter() - start) < 1.2:
                attempts += 1
                ok, frame = capture.read()
                if ok and frame is not None and frame.size > 0:
                    success_count += 1
                    if success_count >= 5:
                        elapsed_ms = int((time.perf_counter() - start) * 1000)
                        return _ProbeResult(True, elapsed_ms)
                else:
                    time.sleep(0.02)
            return _ProbeResult(False)
        finally:
            capture.release()


class CameraHandle:
    def __init__(
        self,
        autopilot: _CameraAutopilotCore,
        selection: CameraSelection,
        stall_timeout: float = STALL_TIMEOUT_SECONDS,
    ) -> None:
        self._autopilot = autopilot
        self._logger = autopilot.logger
        self._selection = selection
        self._stall_timeout = stall_timeout
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._frame_condition = threading.Condition(self._frame_lock)
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_seq = 0
        self._latest_timestamp = 0.0
        self._ema_fps: Optional[float] = None
        self._thread: Optional[threading.Thread] = None
        self._cap_lock = threading.Lock()
        self._cap: Optional[cv2.VideoCapture] = None
        self._stall_recoveries = 0
        self._fatal_error: Optional[str] = None

    @property
    def selection(self) -> CameraSelection:
        return self._selection

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._cap_lock:
            if self._cap is not None:
                try:
                    self._cap.grab()
                    self._cap.grab()
                except Exception:
                    pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        with self._cap_lock:
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None

    def get_frame(self) -> Optional[np.ndarray]:
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()

    def stats(self) -> Dict[str, object]:
        with self._frame_lock:
            fps = self._ema_fps
            last_ts = self._latest_timestamp
            seq = self._latest_seq
        selection = self._selection
        return {
            "device": selection.node,
            "mode": selection.mode.descriptor(),
            "label": selection.label,
            "fps": fps,
            "last_frame_ts": last_ts,
            "sequence": seq,
            "stall_recoveries": self._stall_recoveries,
            "fatal_error": self._fatal_error,
        }

    def frames(self) -> Iterator[Tuple[bool, Optional[np.ndarray]]]:
        last_seq = -1
        while not self._stop_event.is_set():
            frame, seq = self._wait_for_frame(last_seq, timeout=1.0)
            if frame is None:
                if self._fatal_error:
                    yield False, None
                    return
                continue
            last_seq = seq
            yield True, frame

    def _wait_for_frame(
        self, last_seq: int, timeout: float = 1.0
    ) -> Tuple[Optional[np.ndarray], int]:
        deadline = time.time() + timeout
        with self._frame_condition:
            while not self._stop_event.is_set() and self._latest_seq == last_seq:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._frame_condition.wait(timeout=remaining)
            if self._latest_seq == last_seq:
                return None, last_seq
            frame = self._latest_frame.copy() if self._latest_frame is not None else None
            return frame, self._latest_seq

    def _capture_loop(self) -> None:
        selection = self._selection
        stall_timer = time.time()
        capture: Optional[cv2.VideoCapture] = None
        try:
            while not self._stop_event.is_set():
                if capture is None:
                    capture = self._open_capture(selection)
                    if capture is None:
                        try:
                            selection = self._autopilot.recover(selection, "open_failed")
                            self._selection = selection
                            continue
                        except RuntimeError as exc:
                            self._fatal_error = str(exc)
                            self._logger.error("%s", exc)
                            return
                    stall_timer = time.time()
                ok, frame = capture.read()
                now = time.time()
                if ok and frame is not None and frame.size > 0:
                    stall_timer = now
                    self._update_frame(frame, now)
                    continue
                time.sleep(0.01)
                if now - stall_timer > self._stall_timeout:
                    self._logger.warning(
                        "Camera stall detected on %s (%s); attempting recovery",
                        selection.node,
                        selection.mode.descriptor(),
                    )
                    self._release_capture(capture)
                    capture = self._open_capture(selection)
                    if capture is None:
                        self._stall_recoveries += 1
                        try:
                            selection = self._autopilot.recover(selection, "stall")
                            self._selection = selection
                            capture = None
                            stall_timer = time.time()
                        except RuntimeError as exc:
                            self._fatal_error = str(exc)
                            self._logger.error("%s", exc)
                            return
                    else:
                        self._logger.info(
                            "Recovered camera %s with same mode",
                            selection.node,
                        )
                        stall_timer = time.time()
                        continue
        finally:
            if capture is not None:
                self._release_capture(capture)

    def _release_capture(self, capture: cv2.VideoCapture) -> None:
        with self._cap_lock:
            if self._cap is capture:
                self._cap = None
        try:
            capture.release()
        except Exception:
            pass

    def _open_capture(self, selection: CameraSelection) -> Optional[cv2.VideoCapture]:
        capture = cv2.VideoCapture(selection.node, CAPTURE_BACKEND)
        if not capture or not capture.isOpened():
            return None
        _set_if_supported(capture, cv2.CAP_PROP_BUFFERSIZE, 1)
        _set_if_supported(capture, cv2.CAP_PROP_FOURCC, _video_writer_fourcc(selection.mode.fourcc))
        _set_if_supported(capture, cv2.CAP_PROP_FRAME_WIDTH, selection.mode.width)
        _set_if_supported(capture, cv2.CAP_PROP_FRAME_HEIGHT, selection.mode.height)
        _set_if_supported(capture, cv2.CAP_PROP_FPS, selection.mode.fps)
        open_timeout = getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", None)
        read_timeout = getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", None)
        if open_timeout is not None:
            _set_if_supported(capture, open_timeout, 1000)
        if read_timeout is not None:
            _set_if_supported(capture, read_timeout, 1000)
        with self._cap_lock:
            self._cap = capture
        return capture

    def _update_frame(self, frame: np.ndarray, timestamp: float) -> None:
        with self._frame_condition:
            self._latest_frame = frame.copy()
            self._latest_seq += 1
            previous = self._latest_timestamp
            self._latest_timestamp = timestamp
            if previous:
                dt = timestamp - previous
                if dt > 0:
                    fps = 1.0 / dt
                    if self._ema_fps is None:
                        self._ema_fps = fps
                    else:
                        self._ema_fps = 0.8 * self._ema_fps + 0.2 * fps
            self._frame_condition.notify_all()


def open(
    preferred_nodes: Optional[Sequence[str | int]] = None,
    preferred_mode: Optional[CameraMode] = None,
    logger: Optional[logging.Logger] = None,
) -> CameraHandle:
    """Discover cameras and return a handle ready to start streaming."""

    autopilot = _CameraAutopilotCore(
        preferred_nodes=preferred_nodes,
        preferred_mode=preferred_mode,
        logger=logger,
    )
    selection = autopilot.initial_selection()
    return CameraHandle(autopilot=autopilot, selection=selection)


def _preview(duration: float = 5.0) -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    try:
        handle = open()
    except RuntimeError as exc:
        LOGGER.error("%s", exc)
        return 1
    handle.start()
    start = time.time()
    try:
        while time.time() - start < duration:
            stats = handle.stats()
            LOGGER.info(
                "Camera %s (%s) fps=%.2f recoveries=%s",
                stats.get("device"),
                stats.get("mode"),
                stats.get("fps") or 0.0,
                stats.get("stall_recoveries"),
            )
            time.sleep(1.0)
    finally:
        handle.stop()
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Camera Autopilot preview tool")
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Run discovery and print live FPS for five seconds",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=5.0,
        help="Duration of the preview window (seconds)",
    )
    args = parser.parse_args(argv)
    if args.preview:
        return _preview(duration=args.seconds)
    parser.print_help()
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entrypoint
    sys.exit(main())

