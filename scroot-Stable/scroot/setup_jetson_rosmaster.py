#!/usr/bin/env python3
"""Bootstrap a Jetson Nano + Rosmaster environment.

This helper assumes as little as possible: it installs the system packages
required for camera + serial access, sets up a virtual environment that can
see JetPack's prebuilt OpenCV, installs Jetson-compatible PyTorch wheels when
possible, installs the remaining Python requirements, and downloads the
default YOLO weights so the lightweight pilot can start immediately.
"""
from __future__ import annotations

import os
import platform
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional
import venv

ROOT = Path(__file__).resolve().parent
DEFAULT_VENV = ROOT / ".venv"
REQUIREMENTS_FILE = ROOT / "autonomy" / "requirements.txt"
YOLO_MODEL = "yolov8n.pt"


@dataclass
class JetsonWheel:
    torch_url: str
    vision_url: str


JETSON_WHEELS: Dict[str, JetsonWheel] = {
    # JetPack 4.6.x (L4T 32.x) with Python 3.6
    "32": JetsonWheel(
        torch_url="https://developer.download.nvidia.com/compute/redist/jp/v46/pytorch/torch-1.10.0%2Bnv22.1-cp36-cp36m-linux_aarch64.whl",
        vision_url="https://developer.download.nvidia.com/compute/redist/jp/v46/pytorch/torchvision-0.11.1-cp36-cp36m-linux_aarch64.whl",
    ),
    # JetPack 5.1.x (L4T 35.x) with Python 3.8
    "35": JetsonWheel(
        torch_url="https://developer.download.nvidia.com/compute/redist/jp/v51/pytorch/torch-2.1.0%2Bnv23.6-cp38-cp38-linux_aarch64.whl",
        vision_url="https://developer.download.nvidia.com/compute/redist/jp/v51/pytorch/torchvision-0.16.0-cp38-cp38-linux_aarch64.whl",
    ),
}


def run(cmd: list[str], *, env: Optional[dict] = None, check: bool = True) -> None:
    print(f"[jetson] running: {' '.join(cmd)}")
    subprocess.run(cmd, env=env, check=check)


def detect_l4t_version() -> Optional[str]:
    release_path = Path("/etc/nv_tegra_release")
    if not release_path.exists():
        return None
    text = release_path.read_text(encoding="utf-8", errors="ignore")
    match = re.search(r"R(\d+)", text)
    return match.group(1) if match else None


def ensure_system_packages() -> None:
    sudo = [] if os.geteuid() == 0 else ["sudo"]
    packages = [
        "python3-venv",
        "python3-pip",
        "python3-opencv",
        "ffmpeg",
        "v4l-utils",
        "libopenblas-base",
        "libatlas-base-dev",
    ]
    run(sudo + ["apt-get", "update"])
    run(sudo + ["apt-get", "install", "-y", *packages])


def ensure_venv(path: Path) -> Path:
    builder = venv.EnvBuilder(with_pip=True, system_site_packages=True)
    if not path.exists():
        print(f"[jetson] creating virtual environment at {path}")
        builder.create(path)
    else:
        print(f"[jetson] reusing virtual environment at {path}")
    python_bin = path / ("Scripts" if os.name == "nt" else "bin") / "python"
    if not python_bin.exists():
        raise RuntimeError(f"Failed to locate python inside {path}")
    return python_bin


def install_torch_wheels(python_bin: Path, l4t_major: Optional[str]) -> None:
    if l4t_major and l4t_major in JETSON_WHEELS:
        wheel = JETSON_WHEELS[l4t_major]
        try:
            run([str(python_bin), "-m", "pip", "install", wheel.torch_url, wheel.vision_url])
            return
        except subprocess.CalledProcessError:
            print("[jetson] failed to install NVIDIA PyTorch wheels; falling back to pip default")
    run([str(python_bin), "-m", "pip", "install", "torch", "torchvision"])


def install_requirements(python_bin: Path) -> None:
    run([str(python_bin), "-m", "pip", "install", "--upgrade", "pip"])
    if REQUIREMENTS_FILE.exists():
        run([str(python_bin), "-m", "pip", "install", "-r", str(REQUIREMENTS_FILE)])
    else:
        raise FileNotFoundError(f"Missing requirements file at {REQUIREMENTS_FILE}")


def download_yolo(python_bin: Path) -> None:
    script = f"from ultralytics import YOLO; YOLO('{YOLO_MODEL}')"
    run([str(python_bin), "-c", script])


def main() -> None:
    if platform.machine() != "aarch64":
        print("[jetson] Warning: this bootstrapper is intended for Jetson Nano (aarch64)")
    l4t_major = detect_l4t_version()
    if l4t_major:
        print(f"[jetson] detected L4T release R{l4t_major}")
    else:
        print("[jetson] unable to detect L4T release; continuing with generic settings")

    ensure_system_packages()
    python_bin = ensure_venv(DEFAULT_VENV)
    install_torch_wheels(python_bin, l4t_major)
    install_requirements(python_bin)
    download_yolo(python_bin)

    print("\n[jetson] setup complete. Activate the env and launch the pilot:")
    activation = ".venv\\Scripts\\activate" if os.name == "nt" else "source .venv/bin/activate"
    print(f"    {activation}")
    print("    python -m autonomy.simple_cpu_pilot --camera 0 --serial auto")


if __name__ == "__main__":
    main()
