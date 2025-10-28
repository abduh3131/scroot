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
    advisor_model_profile: str
    camera_source: str
    resolution_width: int
    resolution_height: int
    fps: int
    enable_visualization: bool
    enable_advisor: bool
    advisor_mode: str
    safety_mindset: str
    ambient_mode: str
    persona: str
    vehicle_description: str
    vehicle_width_m: float
    vehicle_length_m: float
    vehicle_height_m: float
    vehicle_clearance_margin_m: float
    calibration_reference_distance_m: float
    calibration_reference_pixels: float

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2)

    @classmethod
    def default(cls, model_profile: ModelProfile, dependency_profile: DependencyProfile) -> "AppState":
        return cls(
            model_profile=model_profile.key,
            dependency_profile=dependency_profile.key,
            advisor_model_profile="normal",
            camera_source="0",
            resolution_width=1280,
            resolution_height=720,
            fps=30,
            enable_visualization=True,
            enable_advisor=True,
            advisor_mode="normal",
            safety_mindset="off",
            ambient_mode="on",
            persona="calm_safe",
            vehicle_description="Scooter",
            vehicle_width_m=0.65,
            vehicle_length_m=1.2,
            vehicle_height_m=1.2,
            vehicle_clearance_margin_m=0.2,
            calibration_reference_distance_m=2.0,
            calibration_reference_pixels=220.0,
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
        data.setdefault("advisor_mode", "normal")
        data.setdefault("advisor_model_profile", "normal")
        data.setdefault("safety_mindset", "off")
        data.setdefault("ambient_mode", "on")
        data.setdefault("persona", "calm_safe")
        data.setdefault("vehicle_description", "Scooter")
        data.setdefault("vehicle_width_m", 0.65)
        data.setdefault("vehicle_length_m", 1.2)
        data.setdefault("vehicle_height_m", 1.2)
        data.setdefault("vehicle_clearance_margin_m", 0.2)
        data.setdefault("calibration_reference_distance_m", 2.0)
        data.setdefault("calibration_reference_pixels", 220.0)
        return AppState(**data)

    def save_hardware(self, profile: HardwareProfile) -> None:
        from autonomy.app.hardware import save_hardware_profile

        save_hardware_profile(profile, self.hardware_path)

    def load_hardware(self) -> Optional[HardwareProfile]:
        from autonomy.app.hardware import load_hardware_profile

        return load_hardware_profile(self.hardware_path)
