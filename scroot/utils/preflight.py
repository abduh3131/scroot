"""Preflight helpers executed before launching the GUI."""

from __future__ import annotations

import subprocess
from typing import Callable


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)


def lock_v4l2_mode(
    device: str = "/dev/video0",
    width: int = 640,
    height: int = 480,
    fps: int = 20,
    log: Callable[[str], None] = print,
) -> None:
    """Best-effort MJPEG mode lock via ``v4l2-ctl`` if available."""

    which = _run(["which", "v4l2-ctl"])
    if which.returncode != 0:
        log("[preflight] v4l2-ctl not found; skipping format lock.")
        return

    fmt_cmd = [
        "v4l2-ctl",
        "-d",
        device,
        f"--set-fmt-video=width={width},height={height},pixelformat=MJPG",
    ]
    fmt = _run(fmt_cmd)
    if fmt.returncode == 0:
        log(f"[preflight] Locked {device} to MJPG {width}x{height}.")
    else:
        log(f"[preflight] Could not set fmt on {device}: {fmt.stderr.strip() or fmt.stdout.strip()}")

    parm_cmd = ["v4l2-ctl", "-d", device, f"--set-parm={fps}"]
    parm = _run(parm_cmd)
    if parm.returncode == 0:
        log(f"[preflight] Requested {fps} fps on {device}.")
    else:
        log(f"[preflight] Could not set fps on {device}: {parm.stderr.strip() or parm.stdout.strip()}")

    # Permissions hint for WSL: best-effort, ignore failures
    _run(["sudo", "chmod", "a+rw", device])


__all__ = ["lock_v4l2_mode"]
