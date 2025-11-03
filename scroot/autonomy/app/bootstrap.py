"""Bootstrap helpers to prepare the GUI runtime automatically."""

from __future__ import annotations

from typing import Callable, Optional

from autonomy.app.environment import (
    install_dependencies,
    load_environment_status,
    prepare_advisor_environment,
    resolve_environment_plan,
)
from autonomy.app.hardware import detect_hardware
from autonomy.app.profiles import ADVISOR_MODEL_PROFILES, DEPENDENCY_PROFILES, recommend_profiles
from autonomy.app.state import AppState, AppStateManager


def _default_logger(message: str) -> None:
    print(f"[bootstrap] {message}")


def auto_prepare_environment(logger: Optional[Callable[[str], None]] = None) -> None:
    """Ensure hardware is profiled and dependencies are installed before launching the GUI."""

    log = logger or _default_logger

    state_manager = AppStateManager()

    log("Scanning host hardware and operating system...")
    cached_hardware = state_manager.load_hardware()
    hardware = detect_hardware()
    state_manager.save_hardware(hardware)

    if cached_hardware and cached_hardware != hardware:
        log("Hardware profile changed since last launch; refreshing recommendations.")

    distro_label = hardware.distro_name or hardware.os_version
    log(
        "Environment detected: %s (%s variant=%s)" %
        (hardware.os_name, distro_label, hardware.environment)
    )
    log(
        "CPU=%s cores=%s memory=%sGB" %
        (hardware.cpu_model, hardware.cpu_count, hardware.total_memory_gb)
    )
    if hardware.gpu_name:
        log(f"GPU={hardware.gpu_name} (CUDA={'yes' if hardware.has_cuda else 'no'})")
    else:
        log(f"GPU=none (CUDA={'yes' if hardware.has_cuda else 'no'})")
    log(f"Compute tier classified as '{hardware.compute_tier}'.")
    log("Profile data cached to speed up future launches.")

    recommended_model, recommended_dep, recommended_advisor = recommend_profiles(hardware)

    app_state = state_manager.load_state()
    if app_state is None:
        log("Initializing application state with recommended defaults.")
        app_state = AppState.default(
            recommended_model,
            recommended_dep,
            recommended_advisor.key,
        )
        state_manager.save_state(app_state)
    else:
        log("Loaded existing application state.")
        if app_state.dependency_profile not in DEPENDENCY_PROFILES:
            log(
                "Stored dependency profile is unknown; switching to recommended profile "
                f"'{recommended_dep.key}'."
            )
            app_state.dependency_profile = recommended_dep.key
            state_manager.save_state(app_state)
        if app_state.advisor_model_profile not in ADVISOR_MODEL_PROFILES:
            log(
                "Stored advisor profile is unknown; switching to recommended profile "
                f"'{recommended_advisor.key}'."
            )
            app_state.advisor_model_profile = recommended_advisor.key
            state_manager.save_state(app_state)

    dependency_profile = DEPENDENCY_PROFILES[app_state.dependency_profile]

    status = load_environment_status()
    plan = resolve_environment_plan(dependency_profile, status=status)
    if plan.venv_path:
        log(f"Using managed virtual environment at {plan.venv_path}.")
    else:
        log(f"Using interpreter {plan.python_executable} for dependency management.")
    if status and status.matches_plan(plan):
        log(
            "Dependency profile '%s' already installed (last setup %s)."
            % (dependency_profile.key, status.installed_at or "unknown")
        )
        log(f"See log output at {status.log_path}.")
        return

    log(f"Installing dependency profile '{dependency_profile.key}'...")
    report = install_dependencies(plan, logger=log)
    if report.error:
        raise RuntimeError(
            "Automatic dependency installation failed; "
            f"check {report.log_path} for details: {report.error}"
        )
    log("Dependency installation completed successfully.")
    selected_advisor = ADVISOR_MODEL_PROFILES.get(
        app_state.advisor_model_profile, recommended_advisor
    )
    prepare_advisor_environment(selected_advisor, hardware, log)

