"""Advisor backends for interpreting scene context."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np

try:  # pragma: no cover - optional for lightweight setups
    import torch
    from transformers import (
        AutoModelForSeq2SeqLM,
        AutoTokenizer,
        BlipForConditionalGeneration,
        BlipProcessor,
    )
    _HAVE_TRANSFORMERS = True
except ImportError:  # pragma: no cover - handled dynamically
    torch = None  # type: ignore
    AutoModelForSeq2SeqLM = None  # type: ignore
    AutoTokenizer = None  # type: ignore
    BlipForConditionalGeneration = None  # type: ignore
    BlipProcessor = None  # type: ignore
    _HAVE_TRANSFORMERS = False

from autonomy.utils.data_structures import (
    AdvisorDirective,
    HighLevelCommand,
    NavigationDecision,
    PerceptionSummary,
)


@dataclass
class AdvisorConfig:
    """Configuration for the situational advisor."""

    backend: str = "vlm"
    image_model: str = "Salesforce/blip-image-captioning-base"
    language_model: str = "google/flan-t5-small"
    device: Optional[str] = None
    max_new_tokens: int = 64
    stop_hazard_threshold: float = 0.85
    enforce_traffic_classes: tuple[str, ...] = ("stop sign", "traffic light", "person", "bicycle")
    system_prompt: str = (
        "You are a safety co-pilot for a self-driving scooter. Provide concise driving directives "
        "that obey pedestrian and micromobility rules. Prefer shoulder, sidewalk, or bike lane travel, "
        "yield to pedestrians, and stop for hazards."
    )


class BaseAdvisor:
    def analyze(
        self,
        frame: np.ndarray,
        perception: PerceptionSummary,
        navigation: NavigationDecision,
        command: Optional[HighLevelCommand],
    ) -> AdvisorDirective:
        raise NotImplementedError


class SituationalAdvisor(BaseAdvisor):
    """Vision-language advisor that inspects frames and provides driving directives."""

    def __init__(self, config: AdvisorConfig | None = None) -> None:
        if not _HAVE_TRANSFORMERS:
            raise ImportError(
                "Transformer-based advisor backend requested but the required packages are missing. "
                "Install 'torch' and 'transformers' or switch to the lightweight advisor."
            )
        self.config = config or AdvisorConfig()
        self._device = self.config.device or ("cuda" if torch.cuda.is_available() else "cpu")

        logging.info("Loading advisor VLM '%s'", self.config.image_model)
        self._vlm_processor = BlipProcessor.from_pretrained(self.config.image_model)
        self._vlm = BlipForConditionalGeneration.from_pretrained(self.config.image_model)
        self._vlm.to(self._device)

        logging.info("Loading advisor LLM '%s'", self.config.language_model)
        self._llm_tokenizer = AutoTokenizer.from_pretrained(self.config.language_model)
        self._llm = AutoModelForSeq2SeqLM.from_pretrained(self.config.language_model)
        self._llm.to(self._device)

    def analyze(
        self,
        frame: np.ndarray,
        perception: PerceptionSummary,
        navigation: NavigationDecision,
        command: Optional[HighLevelCommand],
    ) -> AdvisorDirective:
        """Generate a textual directive and determine if an override is needed."""

        caption = self._caption_scene(frame)
        prompt = self._build_prompt(caption, perception, navigation, command)
        text = self._generate_directive(prompt)
        should_stop = navigation.hazard_level >= self.config.stop_hazard_threshold
        enforced_stop = should_stop or self._requires_compliance(perception)

        return AdvisorDirective(
            directive=text.strip(),
            enforced_stop=bool(enforced_stop),
            caption=caption,
        )

    def _caption_scene(self, frame: np.ndarray) -> str:
        inputs = self._vlm_processor(images=frame, return_tensors="pt").to(self._device)
        outputs = self._vlm.generate(**inputs, max_new_tokens=40)
        caption = self._vlm_processor.batch_decode(outputs, skip_special_tokens=True)[0]
        logging.debug("Scene caption: %s", caption)
        return caption

    def _generate_directive(self, prompt: str) -> str:
        tokens = self._llm_tokenizer(prompt, return_tensors="pt").to(self._device)
        outputs = self._llm.generate(
            **tokens,
            max_new_tokens=self.config.max_new_tokens,
            do_sample=False,
            pad_token_id=self._llm_tokenizer.eos_token_id,
        )
        text = self._llm_tokenizer.decode(outputs[0], skip_special_tokens=True)
        logging.debug("Advisor directive: %s", text)
        return text

    def _requires_compliance(self, perception: PerceptionSummary) -> bool:
        for obj in perception.objects:
            if obj.label.lower() in self.config.enforce_traffic_classes:
                logging.debug("Compliance stop triggered by %s", obj.label)
                return True
        return False

    def _build_prompt(
        self,
        caption: str,
        perception: PerceptionSummary,
        navigation: NavigationDecision,
        command: Optional[HighLevelCommand],
    ) -> str:
        obstacle_summary = self._summarize_obstacles(perception.objects)
        goal_text = command.raw_text if command else "No explicit goal."
        prompt = (
            f"{self.config.system_prompt}\n"
            f"Scene: {caption}.\n"
            f"Detections: {obstacle_summary}.\n"
            f"Navigation bias: {navigation.steering_bias:+.2f}, desired_speed={navigation.desired_speed:.2f} m/s, "
            f"hazard_level={navigation.hazard_level:.2f}.\n"
            f"Operator goal: {goal_text}.\n"
            "Respond with a short imperative command like 'slow down', 'stop', or 'use the bike lane'."
        )
        return prompt

    @staticmethod
    def _summarize_obstacles(objects: Iterable) -> str:
        if not objects:
            return "no obstacles detected"
        label_counts: dict[str, int] = {}
        for obj in objects:
            label_counts[obj.label] = label_counts.get(obj.label, 0) + 1
        parts = [f"{count}x {label}" for label, count in sorted(label_counts.items())]
        return ", ".join(parts)


def create_advisor(config: AdvisorConfig) -> BaseAdvisor:
    """Instantiate the requested advisor backend."""

    backend = (config.backend or "vlm").lower()
    if backend == "lite":
        from autonomy.ai.lite_advisor import LiteSituationalAdvisor

        return LiteSituationalAdvisor(config)
    if backend in {"vlm", "default"}:
        if not _HAVE_TRANSFORMERS:
            logging.warning(
                "Transformer-based advisor requested but dependencies are missing; "
                "falling back to the lightweight advisor."
            )
            from autonomy.ai.lite_advisor import LiteSituationalAdvisor

            config.backend = "lite"
            return LiteSituationalAdvisor(config)
        return SituationalAdvisor(config)
    raise ValueError(f"Unknown advisor backend '{config.backend}'")


__all__ = ["AdvisorConfig", "SituationalAdvisor", "create_advisor"]
