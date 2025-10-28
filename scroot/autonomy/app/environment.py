"""Environment preparation utilities for the scooter app."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional

from autonomy.app.profiles import DependencyProfile


STATUS_PATH = Path("config") / "environment_status.json"
DEFAULT_VENV_PATH = Path(".venv")


@dataclass(slots=True)
class EnvironmentPlan:
    dependency_profile: DependencyProfile
    python_executable: str = sys.executable
    extra_args: tuple[str, ...] = ()
    venv_path: Optional[Path] = None

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        payload["dependency_profile"] = self.dependency_profile.key
        payload["extra_args"] = list(self.extra_args)
        if self.venv_path is not None:
            payload["venv_path"] = str(self.venv_path)
        else:
            payload["venv_path"] = None
        return payload


@dataclass(slots=True)
class EnvironmentReport:
    executed: bool
    log_path: Path
    plan: EnvironmentPlan
    error: Optional[str] = None


@dataclass(slots=True)
class EnvironmentStatus:
    profile_key: str
    python_executable: str
    extra_args: tuple[str, ...]
    installed_at: str
    log_path: Path
    venv_path: Optional[Path] = None

    def to_json(self) -> str:
        payload = {
            "profile_key": self.profile_key,
            "python_executable": self.python_executable,
            "extra_args": list(self.extra_args),
            "installed_at": self.installed_at,
            "log_path": str(self.log_path),
            "venv_path": str(self.venv_path) if self.venv_path else None,
        }
        return json.dumps(payload, indent=2)

    def matches_plan(self, plan: EnvironmentPlan) -> bool:
        return (
            self.profile_key == plan.dependency_profile.key
            and self.python_executable == plan.python_executable
            and tuple(self.extra_args) == tuple(plan.extra_args)
            and (
                (self.venv_path is None and plan.venv_path is None)
                or (
                    self.venv_path is not None
                    and plan.venv_path is not None
                    and Path(self.venv_path) == plan.venv_path
                )
            )
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
        venv_path=Path(data["venv_path"]) if data.get("venv_path") else None,
    )


def save_environment_status(status: EnvironmentStatus, path: Path = STATUS_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(status.to_json(), encoding="utf-8")


def _python_from_venv(venv_path: Path) -> Optional[Path]:
    if not venv_path.exists():
        return None
    if os.name == "nt":
        candidate = venv_path / "Scripts" / "python.exe"
    else:
        candidate = venv_path / "bin" / "python"
    return candidate if candidate.exists() else None


def resolve_environment_plan(
    dependency_profile: DependencyProfile,
    status: Optional[EnvironmentStatus] = None,
) -> EnvironmentPlan:
    """Build an installation plan that reuses any existing managed virtualenv."""

    if status and status.python_executable and Path(status.python_executable).exists():
        python_exec = status.python_executable
        venv_path = status.venv_path
    else:
        venv_path = status.venv_path if status else None
        if venv_path:
            python_candidate = _python_from_venv(venv_path)
            if python_candidate:
                python_exec = str(python_candidate)
            else:
                python_exec = sys.executable
                venv_path = None
        else:
            python_candidate = _python_from_venv(DEFAULT_VENV_PATH)
            if python_candidate:
                python_exec = str(python_candidate)
                venv_path = DEFAULT_VENV_PATH
            else:
                python_exec = sys.executable
                venv_path = None

    return EnvironmentPlan(
        dependency_profile=dependency_profile,
        python_executable=python_exec,
        extra_args=dependency_profile.pip_args,
        venv_path=venv_path,
    )


def _write_plan_header(handle, plan: EnvironmentPlan, separator: str = "") -> None:
    if separator:
        handle.write(separator)
    handle.write(json.dumps(plan.to_dict(), indent=2))
    handle.write("\n\n")
    handle.flush()


def _create_virtualenv(target: Path, logger: Callable[[str], None]) -> Path:
    logger(f"Creating dedicated virtual environment at {target}...")
    if not target.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        subprocess.check_call([sys.executable, "-m", "venv", str(target)])
    python_path = _python_from_venv(target)
    if python_path is None:
        raise RuntimeError(f"Failed to locate python executable in virtualenv {target}")
    logger(f"Virtual environment ready: {python_path}")
    subprocess.check_call([str(python_path), "-m", "pip", "install", "--upgrade", "pip"])
    return python_path


def _run_pip_install(
    plan: EnvironmentPlan,
    log_path: Path,
    logger: Callable[[str], None],
    header_mode: str,
) -> tuple[bool, Optional[str]]:
    command = [plan.python_executable, "-m", "pip", "install"]
    if plan.extra_args:
        command.extend(plan.extra_args)
    command.append("--upgrade")
    command.extend(plan.dependency_profile.requirements)

    logger(f"Running: {' '.join(command)}")

    mode = "w" if header_mode == "write" else "a"
    with log_path.open(mode, encoding="utf-8") as handle:
        if header_mode == "write":
            _write_plan_header(handle, plan)
        else:
            _write_plan_header(handle, plan, separator="\n--- retry with updated plan ---\n")
        try:
            subprocess.check_call(command, stdout=handle, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as exc:
            logger(f"Dependency installation failed: {exc}")
            return False, str(exc)
    return True, None


def install_dependencies(plan: EnvironmentPlan, logger: Callable[[str], None]) -> EnvironmentReport:
    """Install dependencies using pip for the selected profile."""

    log_path = Path("logs") / "setup_install.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    success, error = _run_pip_install(plan, log_path, logger, header_mode="write")

    if not success:
        try:
            log_contents = log_path.read_text(encoding="utf-8")
        except OSError:
            log_contents = ""
        pep668_error = "externally-managed-environment" in log_contents
        pep668_error = pep668_error or "externally managed environment" in log_contents
        if pep668_error and plan.venv_path is None:
            try:
                python_path = _create_virtualenv(DEFAULT_VENV_PATH, logger)
            except subprocess.CalledProcessError as exc:
                return EnvironmentReport(
                    executed=False,
                    log_path=log_path,
                    plan=plan,
                    error=f"Failed to create virtual environment: {exc}",
                )
            except RuntimeError as exc:
                return EnvironmentReport(
                    executed=False,
                    log_path=log_path,
                    plan=plan,
                    error=str(exc),
                )

            new_plan = EnvironmentPlan(
                dependency_profile=plan.dependency_profile,
                python_executable=str(python_path),
                extra_args=plan.extra_args,
                venv_path=DEFAULT_VENV_PATH,
            )
            success, error = _run_pip_install(
                new_plan, log_path, logger, header_mode="append"
            )
            if not success:
                return EnvironmentReport(
                    executed=False,
                    log_path=log_path,
                    plan=new_plan,
                    error=error,
                )
            plan = new_plan
        else:
            return EnvironmentReport(
                executed=False,
                log_path=log_path,
                plan=plan,
                error=error,
            )

    logger("Dependency installation complete.")
    status = EnvironmentStatus(
        profile_key=plan.dependency_profile.key,
        python_executable=plan.python_executable,
        extra_args=plan.extra_args,
        installed_at=datetime.now(timezone.utc).isoformat(),
        log_path=log_path,
        venv_path=plan.venv_path,
    )
    save_environment_status(status)

    return EnvironmentReport(executed=True, log_path=log_path, plan=plan)
