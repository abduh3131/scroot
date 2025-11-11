"""Robust OpenCV camera helper tuned for WSL2 + usbip webcams."""

from __future__ import annotations

import os
import time
from typing import Callable, List, Sequence

import cv2


CameraChoice = int | str


def _normalize_choices(values: Sequence[CameraChoice]) -> List[CameraChoice]:
    seen: set[CameraChoice] = set()
    ordered: List[CameraChoice] = []
    for value in values:
        if isinstance(value, str) and value.isdigit():
            normalized: CameraChoice = int(value)
        else:
            normalized = value
        if normalized in seen:
            continue
        seen.add(normalized)
        ordered.append(normalized)
    return ordered


def open_camera_robust(
    prefer_indices: Sequence[int] = (0, 1, 2, 3),
    prefer_paths: Sequence[str] = ("/dev/video0", "/dev/video1", "/dev/video2"),
    width: int = int(os.getenv("CAMERA_WIDTH", "640")),
    height: int = int(os.getenv("CAMERA_HEIGHT", "480")),
    fps: int = int(os.getenv("CAMERA_FPS", "20")),
    prefer_mjpeg: bool = True,
    backend: int = cv2.CAP_V4L2,
    log: Callable[[str], None] = print,
    allow_fallback: bool = True,
) -> cv2.VideoCapture:
    """Open a webcam using V4L2 with resilient probing.

    Parameters honour environment overrides for device selection and resolution.
    If the camera cannot be opened, ``RuntimeError`` is raised with actionable hints.
    """

    os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_LIST", "v4l2")

    choices: List[CameraChoice] = []
    tried: List[CameraChoice] = []

    index_env = os.getenv("CAMERA_DEVICE_INDEX")
    if index_env is not None:
        try:
            choices.append(int(index_env))
        except ValueError:
            log(f"[camera] Ignoring invalid CAMERA_DEVICE_INDEX={index_env!r}")

    if allow_fallback or not choices:
        choices.extend(prefer_indices)

    device_env = os.getenv("CAMERA_DEVICE")
    if device_env:
        choices.append(device_env)

    if allow_fallback or not device_env:
        choices.extend(prefer_paths)

    # Remove duplicates while preserving order
    choices = _normalize_choices(choices)

    fourcc_code = cv2.VideoWriter_fourcc(*"MJPG") if prefer_mjpeg else 0
    deadline = 1.2  # seconds per candidate

    for candidate in choices:
        if candidate in tried:
            continue
        tried.append(candidate)

        # Skip non-existent device paths when a path string is provided
        if isinstance(candidate, str) and not os.path.exists(candidate):
            log(f"[camera] Skipping missing device {candidate}.")
            continue

        try:
            cap = cv2.VideoCapture(candidate, backend)
        except Exception as exc:  # pragma: no cover - backend specific failure
            log(f"[camera] Failed {candidate}: {exc}")
            continue

        if not cap or not cap.isOpened():
            log(f"[camera] Failed to open {candidate} with backend={backend}.")
            cap.release()
            continue

        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if prefer_mjpeg:
                cap.set(cv2.CAP_PROP_FOURCC, fourcc_code)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
            cap.set(cv2.CAP_PROP_FPS, float(fps))

            success_count = 0
            attempts = 0
            start = time.perf_counter()
            while time.perf_counter() - start < deadline and attempts < 60:
                attempts += 1
                ok, frame = cap.read()
                if ok and frame is not None and frame.size > 0:
                    success_count += 1
                    if success_count >= 5:
                        log(
                            f"[camera] Using {candidate} @ {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                            f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}@~{int(cap.get(cv2.CAP_PROP_FPS))},"
                            f" MJPEG={prefer_mjpeg}"
                        )
                        return cap
                else:
                    time.sleep(0.02)
            log(
                f"[camera] {candidate} produced only {success_count}/5 frames during probe; "
                "releasing handle."
            )
            cap.release()
        except Exception as exc:  # pragma: no cover - release for robustness
            cap.release()
            log(f"[camera] Probe error on {candidate}: {exc}")
            continue

    raise RuntimeError(
        "No webcam delivered frames. Tried "
        f"{tried}. Hints: ensure usbipd attach --wsl, close Teams/Zoom/OBS/Camera apps, "
        "run v4l2-ctl to lock MJPEG (640x480@20), or try a different device index."
    )


__all__ = ["open_camera_robust"]
