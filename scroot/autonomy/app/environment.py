"""Environment preparation utilities for the scooter app."""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Optional

from autonomy.app.profiles import DependencyProfile


@dataclass(slots=True)
class EnvironmentPlan:
    dependency_profile: DependencyProfile
    python_executable: str = sys.executable
    extra_args: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        payload["dependency_profile"] = self.dependency_profile.key
        return payload


@dataclass(slots=True)
class EnvironmentReport:
    executed: bool
    log_path: Path
    error: Optional[str] = None


def install_dependencies(plan: EnvironmentPlan, logger: Callable[[str], None]) -> EnvironmentReport:
    """Install dependencies using pip for the selected profile."""

    log_path = Path("logs") / "setup_install.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)

    command = [plan.python_executable, "-m", "pip", "install"]
    if plan.extra_args:
        command.extend(plan.extra_args)
    command.append("--upgrade")
    command.extend(plan.dependency_profile.requirements)

    logger(f"Running: {' '.join(command)}")

    with log_path.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(plan.to_dict(), indent=2))
        handle.write("\n\n")
        handle.flush()
        try:
            subprocess.check_call(command, stdout=handle, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as exc:  # pragma: no cover - depends on environment
            logger(f"Dependency installation failed: {exc}")
            return EnvironmentReport(executed=False, log_path=log_path, error=str(exc))

    logger("Dependency installation complete.")
    return EnvironmentReport(executed=True, log_path=log_path)
