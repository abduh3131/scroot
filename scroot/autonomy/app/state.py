"""Application state management for the GUI."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

from autonomy.app.hardware import HardwareProfile
from autonomy.app.profiles import DependencyProfile, ModelProfile

APP_STATE_PATH = Path("config") / "app_state.json"
HARDWARE_CACHE_PATH = Path("config") / "hardware_profile.json"


@dataclass(slots=True)
class AppState:
    model_profile: str
    dependency_profile: str
    camera_source: str
    resolution_width: int
    resolution_height: int
    fps: int
    enable_visualization: bool
    enable_advisor: bool

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2)

    @classmethod
    def default(cls, model_profile: ModelProfile, dependency_profile: DependencyProfile) -> "AppState":
        return cls(
            model_profile=model_profile.key,
            dependency_profile=dependency_profile.key,
            camera_source="0",
            resolution_width=1280,
            resolution_height=720,
            fps=30,
            enable_visualization=True,
            enable_advisor=True,
        )


class AppStateManager:
    def __init__(self) -> None:
        self.state_path = APP_STATE_PATH
        self.hardware_path = HARDWARE_CACHE_PATH
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        self.hardware_path.parent.mkdir(parents=True, exist_ok=True)

    def save_state(self, state: AppState) -> None:
        self.state_path.write_text(state.to_json(), encoding="utf-8")

    def load_state(self) -> Optional[AppState]:
        if not self.state_path.exists():
            return None
        data = json.loads(self.state_path.read_text(encoding="utf-8"))
        return AppState(**data)

    def save_hardware(self, profile: HardwareProfile) -> None:
        from autonomy.app.hardware import save_hardware_profile

        save_hardware_profile(profile, self.hardware_path)

    def load_hardware(self) -> Optional[HardwareProfile]:
        from autonomy.app.hardware import load_hardware_profile

        return load_hardware_profile(self.hardware_path)
