"""GUI application entry point for the scooter autonomy stack."""

from __future__ import annotations

from typing import Callable, Optional

from .bootstrap import auto_prepare_environment
from .gui import launch_app as _launch_app


def launch_app(*, auto_prepare: bool = True, logger: Optional[Callable[[str], None]] = None) -> None:
    """Launch the scooter autonomy GUI, optionally preparing the environment first."""

    if auto_prepare:
        auto_prepare_environment(logger=logger)
    _launch_app()


__all__ = ["launch_app", "auto_prepare_environment"]
