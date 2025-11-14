"""Hardware and operating system detection utilities."""

from __future__ import annotations

import json
import os
import platform
import shutil
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional, Tuple

try:  # Optional dependency that we include in requirements
    import psutil
except Exception:  # pragma: no cover - psutil import error handled downstream
    psutil = None  # type: ignore

try:  # torch might not be installed during setup
    import torch
except Exception:  # pragma: no cover - torch import error handled downstream
    torch = None  # type: ignore


@dataclass
class HardwareProfile:
    """Summary of the runtime environment."""

    os_name: str
    os_version: str
    cpu_model: str
    cpu_count: int
    total_memory_gb: float
    has_cuda: bool
    gpu_name: Optional[str]
    compute_tier: str
    distro_name: str = ""
    environment: str = "unknown"

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2)


def _read_cpu_model() -> str:
    system = platform.system()
    if system == "Linux":
        cpuinfo = Path("/proc/cpuinfo")
        if cpuinfo.exists():
            for line in cpuinfo.read_text(encoding="utf-8", errors="ignore").splitlines():
                if "model name" in line:
                    return line.split(":", 1)[-1].strip()
    if system == "Darwin":  # macOS
        try:
            output = subprocess.check_output(["sysctl", "-n", "machdep.cpu.brand_string"], text=True)
            return output.strip()
        except Exception:  # pragma: no cover - platform specific
            return platform.processor() or "Unknown CPU"
    if system == "Windows":
        try:
            output = subprocess.check_output(["wmic", "cpu", "get", "name"], text=True)
            lines = [line.strip() for line in output.splitlines() if line.strip()]
            if len(lines) > 1:
                return lines[1]
        except Exception:  # pragma: no cover - platform specific
            return platform.processor() or "Unknown CPU"
    return platform.processor() or "Unknown CPU"


def _detect_gpu() -> Tuple[bool, Optional[str]]:
    if torch is not None and getattr(torch, "cuda", None):
        try:
            if torch.cuda.is_available():
                name = torch.cuda.get_device_name(0)
                return True, name
        except Exception:  # pragma: no cover - runtime specific
            return False, None
    # Fallback: check for NVIDIA SMI
    if shutil.which("nvidia-smi"):
        try:
            output = subprocess.check_output(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], text=True)
            first_line = output.splitlines()[0].strip()
            if first_line:
                return True, first_line
        except Exception:  # pragma: no cover - command might fail
            return True, "NVIDIA GPU"
    return False, None


def _compute_memory_gb() -> float:
    if psutil is not None:
        try:
            return round(psutil.virtual_memory().total / (1024**3), 2)
        except Exception:  # pragma: no cover - psutil error
            pass
    return 0.0


def classify_compute_tier(cpu_count: int, memory_gb: float, has_cuda: bool) -> str:
    """Return a friendly compute tier label."""

    if has_cuda and memory_gb >= 16 and cpu_count >= 8:
        return "performance"
    if memory_gb >= 12 and cpu_count >= 8:
        return "balanced"
    if memory_gb >= 8 and cpu_count >= 4:
        return "standard"
    return "lightweight"


def _detect_linux_distribution() -> str:
    os_release = Path("/etc/os-release")
    if os_release.exists():
        data: dict[str, str] = {}
        for line in os_release.read_text(encoding="utf-8", errors="ignore").splitlines():
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            data[key.strip()] = value.strip().strip('"')
        name = data.get("PRETTY_NAME")
        if name:
            return name
        if data.get("NAME") and data.get("VERSION_ID"):
            return f"{data['NAME']} {data['VERSION_ID']}"
        if data.get("NAME"):
            return data["NAME"]
    return ""


def _detect_environment(os_name: str, distro_name: str) -> str:
    if os_name == "Linux":
        uname = platform.uname()
        release = platform.release().lower()
        if os.environ.get("WSL_DISTRO_NAME") or "microsoft" in release:
            return "wsl"
        machine = uname.machine.lower()
        if "tegra" in machine or "jetson" in machine:
            return "jetson"
        if Path("/etc/nv_tegra_release").exists():
            return "jetson"
        if "jetson" in distro_name.lower():
            return "jetson"
        return "linux_native"
    if os_name == "Windows":
        return "windows"
    if os_name == "Darwin":
        return "mac"
    return os_name.lower() or "unknown"


def detect_hardware() -> HardwareProfile:
    """Gather CPU, memory, GPU, and OS information."""

    os_name = platform.system()
    os_version = platform.version()
    cpu_model = _read_cpu_model()
    cpu_count = psutil.cpu_count(logical=True) if psutil else (os.cpu_count() or 1)  # type: ignore[name-defined]
    memory_gb = _compute_memory_gb()
    has_cuda, gpu_name = _detect_gpu()
    tier = classify_compute_tier(cpu_count, memory_gb, has_cuda)
    distro_name = _detect_linux_distribution() if os_name == "Linux" else platform.platform()
    environment = _detect_environment(os_name, distro_name)

    return HardwareProfile(
        os_name=os_name,
        os_version=os_version,
        cpu_model=cpu_model,
        cpu_count=cpu_count,
        total_memory_gb=memory_gb,
        has_cuda=has_cuda,
        gpu_name=gpu_name,
        compute_tier=tier,
        distro_name=distro_name,
        environment=environment,
    )


def save_hardware_profile(profile: HardwareProfile, path: Path) -> None:
    path.write_text(profile.to_json(), encoding="utf-8")


def load_hardware_profile(path: Path) -> Optional[HardwareProfile]:
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    defaults: dict[str, object] = {
        "os_name": platform.system(),
        "os_version": platform.version(),
        "cpu_model": platform.processor() or "Unknown CPU",
        "cpu_count": os.cpu_count() or 1,
        "total_memory_gb": 0.0,
        "has_cuda": False,
        "gpu_name": None,
        "compute_tier": "lightweight",
        "distro_name": "",
        "environment": "unknown",
    }
    defaults.update(data)
    defaults["cpu_count"] = int(defaults.get("cpu_count", 1))
    defaults["total_memory_gb"] = float(defaults.get("total_memory_gb", 0.0))
    return HardwareProfile(**defaults)
