"""Runtime guardrails for heterogeneous hardware environments."""

from __future__ import annotations

import atexit
import faulthandler
import logging
import os
import platform
import threading
from dataclasses import dataclass
from typing import Callable, List, Optional


LOGGER = logging.getLogger("runtime_guard")


@dataclass
class _GuardState:
    applied: bool = False
    device: str = "cpu"
    capability: Optional[str] = None
    reason: Optional[str] = None
    pid: int = 0


_STATE = _GuardState()
_STATE_LOCK = threading.Lock()
_SHUTDOWN_LOCK = threading.Lock()
_SHUTDOWN_CALLBACKS: List[Callable[[], None]] = []
_ATEEXIT_REGISTERED = False


def _detect_cuda_capability() -> tuple[bool, Optional[str], Optional[str]]:
    try:
        import torch  # type: ignore

        if not torch.cuda.is_available():
            return False, None, "cuda_unavailable"
        capability = torch.cuda.get_device_capability(0)
        sm = f"{capability[0]}.{capability[1]}"
        major = capability[0]
        if major < 7:
            return False, sm, "insufficient_sm"
        return True, sm, None
    except Exception:  # pragma: no cover - torch optional
        return False, None, "torch_missing"


def _ensure_atexit_registered() -> None:
    global _ATEEXIT_REGISTERED
    if _ATEEXIT_REGISTERED:
        return

    def _shutdown() -> None:
        with _SHUTDOWN_LOCK:
            callbacks = list(_SHUTDOWN_CALLBACKS)
            _SHUTDOWN_CALLBACKS.clear()
        for callback in callbacks:
            try:
                callback()
            except Exception:  # pragma: no cover - defensive
                LOGGER.exception("Runtime guard shutdown callback failed")

    atexit.register(_shutdown)
    _ATEEXIT_REGISTERED = True


def register_shutdown_callback(callback: Callable[[], None]) -> None:
    """Register a callback that will run exactly once at interpreter exit."""

    _ensure_atexit_registered()
    with _SHUTDOWN_LOCK:
        if callback not in _SHUTDOWN_CALLBACKS:
            _SHUTDOWN_CALLBACKS.append(callback)


def apply_runtime_guard(preferred_device: str = "auto") -> str:
    """Apply runtime guardrails and return the effective compute device."""

    if not faulthandler.is_enabled():
        try:
            faulthandler.enable()
        except Exception:  # pragma: no cover - safety
            pass

    preferred = (preferred_device or "auto").lower()
    with _STATE_LOCK:
        pid = os.getpid()
        if _STATE.applied and _STATE.pid == pid:
            return _STATE.device

        supported, sm, reason = _detect_cuda_capability()
        forced_cpu = os.environ.get("FORCE_CPU", "0") == "1" or preferred == "cpu"
        if preferred == "gpu" and (not supported or forced_cpu):
            LOGGER.warning("GPU mode requested but unavailable; falling back to CPU")
            forced_cpu = True
        if preferred != "gpu" and not forced_cpu and not supported:
            forced_cpu = True

        device = "cuda"
        if forced_cpu or not supported:
            device = "cpu"

        if device == "cpu":
            os.environ["CUDA_VISIBLE_DEVICES"] = ""
            os.environ["ULTRALYTICS_FORCE_CPU"] = "1"
            os.environ.setdefault("OMP_NUM_THREADS", "4")
            os.environ.setdefault("MKL_NUM_THREADS", "4")
            os.environ.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")
            try:
                import torch  # type: ignore

                torch.set_num_threads(int(os.environ.get("OMP_NUM_THREADS", "4")))
            except Exception:  # pragma: no cover - torch optional
                pass
            if sm:
                LOGGER.info("GPU unsupported (SM %s). Running CPU-only.", sm)
            else:
                LOGGER.info("GPU unavailable. Running CPU-only.")
        else:
            LOGGER.info("GPU supported (SM %s). Using CUDA inference.", sm)

        _STATE.applied = True
        _STATE.device = device
        _STATE.capability = sm
        _STATE.reason = reason
        _STATE.pid = pid

        _ensure_atexit_registered()
        return device


def runtime_environment_summary() -> dict[str, str]:
    """Expose basic runtime diagnostics for telemetry logging."""

    is_wsl = "microsoft" in platform.release().lower()
    summary = {
        "platform": platform.platform(),
        "python": platform.python_version(),
        "wsl": "yes" if is_wsl else "no",
        "device": _STATE.device,
    }
    if _STATE.capability:
        summary["sm"] = _STATE.capability
    if _STATE.reason:
        summary["gpu_reason"] = _STATE.reason
    return summary


def current_device() -> str:
    with _STATE_LOCK:
        return _STATE.device


__all__ = [
    "apply_runtime_guard",
    "current_device",
    "register_shutdown_callback",
    "runtime_environment_summary",
]

