"""Profiles for models and dependency stacks based on hardware tiers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

from autonomy.app.hardware import HardwareProfile


@dataclass(slots=True)
class ModelProfile:
    key: str
    label: str
    description: str
    yolo_model: str
    advisor_image_model: str
    advisor_language_model: str


@dataclass(slots=True)
class AdvisorModelProfile:
    key: str
    label: str
    description: str
    advisor_image_model: str
    advisor_language_model: str


@dataclass(slots=True)
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
        description="Optimized for CPUs with ≤4 cores or <8 GB RAM. Uses YOLOv8n and smaller advisor models.",
        yolo_model="yolov8n.pt",
        advisor_image_model="Salesforce/blip-image-captioning-base",
        advisor_language_model="google/flan-t5-small",
    ),
    "standard": ModelProfile(
        key="standard",
        label="Standard",
        description="Balanced profile for mid-tier laptops or Jetson Xavier devices.",
        yolo_model="yolov8s.pt",
        advisor_image_model="Salesforce/blip-image-captioning-large",
        advisor_language_model="google/flan-t5-base",
    ),
    "performance": ModelProfile(
        key="performance",
        label="Performance",
        description="High-end profile using YOLOv8m and FLAN-T5-large for richer guidance.",
        yolo_model="yolov8m.pt",
        advisor_image_model="Salesforce/blip-image-captioning-large",
        advisor_language_model="google/flan-t5-large",
    ),
}


ADVISOR_MODEL_PROFILES: Dict[str, AdvisorModelProfile] = {
    "light": AdvisorModelProfile(
        key="light",
        label="Light",
        description="Fastest response for CPUs/Jetson Nano. Uses compact vision-language models.",
        advisor_image_model="Salesforce/blip-image-captioning-base",
        advisor_language_model="google/flan-t5-small",
    ),
    "normal": AdvisorModelProfile(
        key="normal",
        label="Normal",
        description="Balanced cognition for laptops/Xavier class devices.",
        advisor_image_model="Salesforce/blip-image-captioning-large",
        advisor_language_model="google/flan-t5-base",
    ),
    "heavy": AdvisorModelProfile(
        key="heavy",
        label="Heavy",
        description="Richest narration and reasoning using larger language models (needs strong GPU).",
        advisor_image_model="Salesforce/blip-image-captioning-large",
        advisor_language_model="google/flan-t5-large",
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
            "transformers==4.36.0",
            "accelerate==0.24.1",
            "sentencepiece==0.1.99",
            "safetensors==0.3.2",
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
            "transformers>=4.37.0",
            "accelerate>=0.25.0",
            "sentencepiece>=0.1.99",
            "safetensors>=0.3.1",
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
            "transformers==4.36.0",
            "accelerate==0.24.1",
            "sentencepiece==0.1.99",
            "safetensors==0.3.1",
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
            "transformers>=4.37.0",
            "accelerate>=0.25.0",
            "sentencepiece>=0.1.99",
            "safetensors>=0.3.1",
            "psutil>=5.9.0",
            "xformers>=0.0.23",
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
        model_profile = MODEL_PROFILES["standard"]

    dep_profile = DEPENDENCY_PROFILES["modern"]
    if profile.environment in DEFAULT_DEPENDENCIES_BY_ENVIRONMENT:
        dep_profile = DEFAULT_DEPENDENCIES_BY_ENVIRONMENT[profile.environment]
    elif profile.os_name in DEFAULT_DEPENDENCIES_BY_OS:
        dep_profile = DEFAULT_DEPENDENCIES_BY_OS[profile.os_name]

    if profile.has_cuda and profile.compute_tier == "performance" and profile.environment != "jetson":
        dep_profile = DEPENDENCY_PROFILES["performance"]

    return model_profile, dep_profile
