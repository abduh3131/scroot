"""Runtime utilities for environment guardrails."""

from .runtime_guard import (
    apply_runtime_guard,
    current_device,
    register_shutdown_callback,
    runtime_environment_summary,
)

__all__ = [
    "apply_runtime_guard",
    "current_device",
    "register_shutdown_callback",
    "runtime_environment_summary",
]

