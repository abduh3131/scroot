"""Profiles for models and dependency stacks based on hardware tiers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

from autonomy.app.hardware import HardwareProfile


@dataclass
class ModelProfile:
    key: str
    label: str
    description: str
    yolo_model: str


@dataclass
class LaneProfile:
    key: str
    label: str
    description: str
    min_confidence: float
    smoothing: float
    roi_top_ratio: float = 0.66
    auto_pitch: bool = True
    horizon_offset: float = 0.02
    adaptive_block_size: int = 25
    adaptive_C: float = -8.0


@dataclass
class DependencyProfile:
    key: str
    label: str
    description: str
    requirements: List[str]
    pip_args: Tuple[str, ...] = ()


MODEL_PROFILES: Dict[str, ModelProfile] = {
    "lightweight": ModelProfile(
        key="lightweight",
        label="Lightweight",
        description="Optimized for CPUs with ≤4 cores or <8 GB RAM. Uses YOLOv8n and lower FPS settings.",
        yolo_model="yolov8n.pt",
    ),
    "standard": ModelProfile(
        key="standard",
        label="Standard",
        description="Balanced profile for mid-tier laptops or Jetson Xavier devices.",
        yolo_model="yolov8s.pt",
    ),
    "performance": ModelProfile(
        key="performance",
        label="Performance",
        description="High-end profile using YOLOv8m for richer detections at higher speeds.",
        yolo_model="yolov8m.pt",
    ),
    "jetson_orin": ModelProfile(
        key="jetson_orin",
        label="Jetson Orin Optimized",
        description=(
            "Tailored defaults for NVIDIA Jetson Orin modules balancing throughput and latency. "
            "Uses a tuned YOLOv8s checkpoint with CUDA-friendly parameters."
        ),
        yolo_model="yolov8s.pt",
    ),
}


LANE_PROFILES: Dict[str, LaneProfile] = {
    "precision": LaneProfile(
        key="precision",
        label="Precision",
        description="Higher confidence threshold and heavier smoothing for slow, exact placement.",
        min_confidence=0.45,
        smoothing=0.8,
        roi_top_ratio=0.68,
        auto_pitch=False,
        horizon_offset=0.015,
        adaptive_block_size=29,
    ),
    "balanced": LaneProfile(
        key="balanced",
        label="Balanced",
        description="Default compromise used for mixed urban riding.",
        min_confidence=0.35,
        smoothing=0.6,
        roi_top_ratio=0.66,
        auto_pitch=True,
        horizon_offset=0.02,
        adaptive_block_size=27,
    ),
    "aggressive": LaneProfile(
        key="aggressive",
        label="Agile",
        description="Lower confidence threshold with minimal smoothing for tight, twisty roads.",
        min_confidence=0.25,
        smoothing=0.4,
        roi_top_ratio=0.63,
        auto_pitch=True,
        horizon_offset=0.03,
        adaptive_block_size=23,
        adaptive_C=-10.0,
    ),
}


DEPENDENCY_PROFILES: Dict[str, DependencyProfile] = {
    "legacy": DependencyProfile(
        key="legacy",
        label="Legacy OS Stack",
        description="Targets Ubuntu 20.04 / Windows 10 era systems with conservative versions.",
        requirements=[
            "ultralytics==8.0.196",
            "opencv-python==4.8.0.76",
            "torch==2.0.1",
            "torchvision==0.15.2",
            "psutil>=5.9.0",
        ],
    ),
    "modern": DependencyProfile(
        key="modern",
        label="Modern OS Stack",
        description="Optimized for Ubuntu 22.04+/Windows 11 with current stable libraries.",
        requirements=[
            "ultralytics>=8.1.0",
            "opencv-python>=4.8.0",
            "torch>=2.1.0",
            "torchvision>=0.16.0",
            "psutil>=5.9.0",
        ],
    ),
    "jetson": DependencyProfile(
        key="jetson",
        label="Jetson Ubuntu Stack",
        description="Prepares wheels compatible with NVIDIA Jetson devices running Ubuntu-based JetPack releases.",
        requirements=[
            "ultralytics==8.0.196",
            "opencv-python==4.7.0.72",
            "torch==2.0.0",
            "torchvision==0.15.1",
            "psutil>=5.9.0",
        ],
        pip_args=(
            "--extra-index-url",
            "https://developer.download.nvidia.com/compute/redist/jp/v512",
        ),
    ),
    "performance": DependencyProfile(
        key="performance",
        label="Performance GPU Stack",
        description="Adds torch/torchvision pinned to recent releases; install CUDA-specific wheels manually if required.",
        requirements=[
            "ultralytics>=8.1.0",
            "opencv-python>=4.8.0",
            "torch>=2.1.0",
            "torchvision>=0.16.0",
            "psutil>=5.9.0",
        ],
    ),
}


DEFAULT_MODEL_BY_TIER = {
    "performance": MODEL_PROFILES["performance"],
    "balanced": MODEL_PROFILES["standard"],
    "standard": MODEL_PROFILES["standard"],
    "lightweight": MODEL_PROFILES["lightweight"],
}


DEFAULT_DEPENDENCIES_BY_OS = {
    "Windows": DEPENDENCY_PROFILES["legacy"],
    "Linux": DEPENDENCY_PROFILES["modern"],
    "Darwin": DEPENDENCY_PROFILES["modern"],
}


DEFAULT_DEPENDENCIES_BY_ENVIRONMENT = {
    "jetson": DEPENDENCY_PROFILES["jetson"],
    "wsl": DEPENDENCY_PROFILES["modern"],
    "linux_native": DEPENDENCY_PROFILES["modern"],
    "windows": DEPENDENCY_PROFILES["legacy"],
    "mac": DEPENDENCY_PROFILES["modern"],
}


def recommend_profiles(profile: HardwareProfile) -> tuple[ModelProfile, DependencyProfile]:
    model_profile = DEFAULT_MODEL_BY_TIER.get(profile.compute_tier, MODEL_PROFILES["lightweight"])

    if profile.environment == "jetson":
        gpu_name = (profile.gpu_name or "").lower()
        if "orin" in gpu_name:
            model_profile = MODEL_PROFILES["jetson_orin"]
        else:
            model_profile = MODEL_PROFILES["standard"]

    dep_profile = DEPENDENCY_PROFILES["modern"]
    if profile.environment in DEFAULT_DEPENDENCIES_BY_ENVIRONMENT:
        dep_profile = DEFAULT_DEPENDENCIES_BY_ENVIRONMENT[profile.environment]
    elif profile.os_name in DEFAULT_DEPENDENCIES_BY_OS:
        dep_profile = DEFAULT_DEPENDENCIES_BY_OS[profile.os_name]

    if profile.has_cuda and profile.compute_tier == "performance" and profile.environment != "jetson":
        dep_profile = DEPENDENCY_PROFILES["performance"]

    return model_profile, dep_profile
