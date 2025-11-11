"""Rule-based advisor that avoids heavy model dependencies."""

from __future__ import annotations

import time
from typing import Any, Optional

from autonomy.utils.data_structures import (
    AdvisorDirective,
    HighLevelCommand,
    NavigationDecision,
    PerceptionSummary,
)

try:  # typing-only import to avoid circular at runtime
    from typing import TYPE_CHECKING
except ImportError:  # pragma: no cover
    TYPE_CHECKING = False  # type: ignore

if TYPE_CHECKING:  # pragma: no cover
    from autonomy.ai.advisor import AdvisorConfig, BaseAdvisor
else:  # pragma: no cover
    AdvisorConfig = object  # type: ignore[misc]
    BaseAdvisor = object  # type: ignore[misc]


class LiteSituationalAdvisor(BaseAdvisor):
    """Fast heuristic advisor suitable for CPU-only systems."""

    def __init__(self, config: "AdvisorConfig" | None = None) -> None:
        from autonomy.ai.advisor import AdvisorConfig as _AdvisorConfig  # lazy import

        if config is None or not isinstance(config, _AdvisorConfig):
            config = _AdvisorConfig(backend="lite")
        if getattr(config, "backend", "lite").lower() != "lite":
            config.backend = "lite"  # type: ignore[attr-defined]
        self.config = config
        self._last_summary = ""
        self._last_update = time.time()

    def analyze(
        self,
        frame: Any,
        perception: PerceptionSummary,
        navigation: NavigationDecision,
        command: Optional[HighLevelCommand],
    ) -> AdvisorDirective:
        caption = self._summarize_scene(perception)
        directive, enforced = self._choose_directive(perception, navigation, command)
        return AdvisorDirective(directive=directive, enforced_stop=enforced, caption=caption)

    # ------------------------------------------------------------------
    def _choose_directive(
        self,
        perception: PerceptionSummary,
        navigation: NavigationDecision,
        command: Optional[HighLevelCommand],
    ) -> tuple[str, bool]:
        hazard = navigation.hazard_level
        enforced = False

        if hazard >= self.config.stop_hazard_threshold:
            enforced = True
            return ("Fail-stop: hazard ahead", True)

        blocked = any(
            obj.label.lower() in getattr(self.config, "enforce_traffic_classes", ())
            for obj in perception.objects
        )
        if blocked:
            return ("Stop for protected road user", True)

        if hazard >= 0.5:
            return ("Creep forward with caution", False)

        steer = navigation.steering_bias
        if steer > 0.2:
            advice = "Nudge right to keep center"
        elif steer < -0.2:
            advice = "Nudge left to keep center"
        else:
            advice = "Hold line"

        if command and command.command_type == "navigate_to" and command.target:
            advice = f"Proceed toward {command.target}"

        return advice, enforced

    def _summarize_scene(self, perception: PerceptionSummary) -> str:
        now = time.time()
        if not perception.objects:
            summary = "Scene clear"
        else:
            label_counts: dict[str, int] = {}
            for obj in perception.objects:
                label_counts[obj.label] = label_counts.get(obj.label, 0) + 1
            parts = [f"{count} {label}" for label, count in sorted(label_counts.items())]
            summary = ", ".join(parts)
        # rate limit updates to avoid flicker
        if summary != self._last_summary or (now - self._last_update) > 1.5:
            self._last_summary = summary
            self._last_update = now
        return self._last_summary
