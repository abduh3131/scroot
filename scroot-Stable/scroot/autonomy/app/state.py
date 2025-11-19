"""Application state management for the GUI."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional, Tuple

from autonomy.app.hardware import HardwareProfile
from autonomy.app.profiles import DependencyProfile, ModelProfile

APP_STATE_PATH = Path("config") / "app_state.json"
HARDWARE_CACHE_PATH = Path("config") / "hardware_profile.json"
RUNTIME_CAMERA_DEFAULT = str((Path.home() / "scroot" / "runtime_inputs" / "camera.jpg"))

DEFAULT_LANE_SRC_POINTS: Tuple[Tuple[float, float], ...] = (
    (0.12, 1.0),
    (0.88, 1.0),
    (0.76, 0.74),
    (0.24, 0.74),
)


@dataclass
class AppState:
    model_profile: str
    dependency_profile: str
    lane_profile: str
    camera_source: str
    resolution_width: int
    resolution_height: int
    fps: int
    enable_visualization: bool
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
    acceleration_mode: str = "auto"
    advisor_enabled: bool = True
    lane_src_points: Tuple[Tuple[float, float], ...] = DEFAULT_LANE_SRC_POINTS
    lane_auto_pitch: bool = True
    lane_horizon_offset: float = 0.02

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2)

    @classmethod
    def default(cls, model_profile: ModelProfile, dependency_profile: DependencyProfile) -> "AppState":
        return cls(
            model_profile=model_profile.key,
            dependency_profile=dependency_profile.key,
            lane_profile="balanced",
            camera_source=RUNTIME_CAMERA_DEFAULT,
            resolution_width=1280,
            resolution_height=720,
            fps=30,
            enable_visualization=True,
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
            acceleration_mode="auto",
            advisor_enabled=True,
            lane_src_points=DEFAULT_LANE_SRC_POINTS,
            lane_auto_pitch=True,
            lane_horizon_offset=0.02,
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
        data.pop("enable_advisor", None)
        data.pop("advisor_mode", None)
        data.setdefault("lane_profile", "balanced")
        data.setdefault("camera_source", RUNTIME_CAMERA_DEFAULT)
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
        data.setdefault("acceleration_mode", "auto")
        data.setdefault("advisor_enabled", True)
        data.setdefault("lane_src_points", [list(pt) for pt in DEFAULT_LANE_SRC_POINTS])
        data.setdefault("lane_auto_pitch", True)
        data.setdefault("lane_horizon_offset", 0.02)
        lane_pts = []
        for pt in data.get("lane_src_points", []):
            if (
                isinstance(pt, (list, tuple))
                and len(pt) == 2
                and all(isinstance(coord, (int, float)) for coord in pt)
            ):
                lane_pts.append((float(pt[0]), float(pt[1])))
        if len(lane_pts) == 4:
            data["lane_src_points"] = tuple(lane_pts)
        else:
            data["lane_src_points"] = DEFAULT_LANE_SRC_POINTS
        return AppState(**data)

    def save_hardware(self, profile: HardwareProfile) -> None:
        from autonomy.app.hardware import save_hardware_profile

        save_hardware_profile(profile, self.hardware_path)

    def load_hardware(self) -> Optional[HardwareProfile]:
        from autonomy.app.hardware import load_hardware_profile

        return load_hardware_profile(self.hardware_path)
