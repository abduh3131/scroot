"""Desktop application launcher with WSL camera preflight."""

from __future__ import annotations

import os

from autonomy.app import launch_app
from scroot.utils.preflight import lock_v4l2_mode


def main() -> None:  # pragma: no cover - UI entry
    width = int(os.getenv("CAMERA_WIDTH", "640"))
    height = int(os.getenv("CAMERA_HEIGHT", "480"))
    fps = int(os.getenv("CAMERA_FPS", "20"))

    lock_v4l2_mode("/dev/video0", width, height, fps)

    os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_LIST", "v4l2")

    launch_app(auto_prepare=True)


if __name__ == "__main__":  # pragma: no cover
    main()
