"""Environment preparation utilities for the scooter app."""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional

from autonomy.app.profiles import DependencyProfile


STATUS_PATH = Path("config") / "environment_status.json"


@dataclass(slots=True)
class EnvironmentPlan:
    dependency_profile: DependencyProfile
    python_executable: str = sys.executable
    extra_args: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        payload["dependency_profile"] = self.dependency_profile.key
        payload["extra_args"] = list(self.extra_args)
        return payload


@dataclass(slots=True)
class EnvironmentReport:
    executed: bool
    log_path: Path
    error: Optional[str] = None


@dataclass(slots=True)
class EnvironmentStatus:
    profile_key: str
    python_executable: str
    extra_args: tuple[str, ...]
    installed_at: str
    log_path: Path

    def to_json(self) -> str:
        payload = {
            "profile_key": self.profile_key,
            "python_executable": self.python_executable,
            "extra_args": list(self.extra_args),
            "installed_at": self.installed_at,
            "log_path": str(self.log_path),
        }
        return json.dumps(payload, indent=2)

    def matches_plan(self, plan: EnvironmentPlan) -> bool:
        return (
            self.profile_key == plan.dependency_profile.key
            and self.python_executable == plan.python_executable
            and tuple(self.extra_args) == tuple(plan.extra_args)
        )


def load_environment_status(path: Path = STATUS_PATH) -> Optional[EnvironmentStatus]:
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    return EnvironmentStatus(
        profile_key=data.get("profile_key", ""),
        python_executable=data.get("python_executable", sys.executable),
        extra_args=tuple(data.get("extra_args", [])),
        installed_at=data.get("installed_at", ""),
        log_path=Path(data.get("log_path", "logs/setup_install.log")),
    )


def save_environment_status(status: EnvironmentStatus, path: Path = STATUS_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(status.to_json(), encoding="utf-8")


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
    status = EnvironmentStatus(
        profile_key=plan.dependency_profile.key,
        python_executable=plan.python_executable,
        extra_args=plan.extra_args,
        installed_at=datetime.now(timezone.utc).isoformat(),
        log_path=log_path,
    )
    save_environment_status(status)

    return EnvironmentReport(executed=True, log_path=log_path)
