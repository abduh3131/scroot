#!/usr/bin/env python3
"""Wrapper node that executes the scroot ROS SensorHub bridge."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path


def _candidate_roots() -> list[Path]:
    home = Path.home()
    return [
        home / "scroot" / "scroot",
        home / "scroot" / "scroot-Stable" / "scroot",
        home / "scroot",
        Path(__file__).resolve().parents[3] / "scroot",
    ]


def _import_bridge_module():
    candidates = ["bridge.ai_input_bridge", "bridge.ros_sensor_bridge"]
    for root in _candidate_roots():
        for module_name in candidates:
            bridge_path = root / Path(module_name.replace(".", "/") + ".py")
            if bridge_path.exists():
                if str(root) not in sys.path:
                    sys.path.insert(0, str(root))
                return importlib.import_module(module_name)
    raise RuntimeError(
        "Unable to locate scroot bridge modules. Ensure the repo is cloned in ~/scroot."
    )


def main() -> None:
    module = _import_bridge_module()
    if hasattr(module, "main"):
        module.main()
    else:  # pragma: no cover
        raise RuntimeError("bridge.ai_input_bridge is missing a main() entry point")


if __name__ == "__main__":  # pragma: no cover
    main()
