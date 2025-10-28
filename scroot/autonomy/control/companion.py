"""Human-style narration that mirrors advisor decisions."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict

from autonomy.utils.data_structures import AdvisorReview


PERSONA_TONES: Dict[str, Dict[str, str]] = {
    "calm_safe": {
        "BLOCK": "Fail-stop: {reason}.",
        "AMEND": "Adjusting course: {reason}.",
        "ALLOW": "All clear."
    },
    "smart_scout": {
        "BLOCK": "Hold up! {reason}.",
        "AMEND": "Tweaking path {reason}.",
        "ALLOW": "Scout says go."},
    "playful": {
        "BLOCK": "Whoa there! {reason}!",
        "AMEND": "Little shimmy {reason}.",
        "ALLOW": "Cruising easy."},
}


@dataclass
class RidingCompanion:
    persona: str = "calm_safe"
    min_interval_s: float = 2.0
    _last_ts: float = 0.0

    def narrate(self, review: AdvisorReview) -> str | None:
        now = time.time()
        if now - self._last_ts < self.min_interval_s:
            return None

        tones = PERSONA_TONES.get(self.persona, PERSONA_TONES["calm_safe"])
        template = tones.get(review.verdict.value, tones["ALLOW"])

        primary_reason = review.reason_tags[0] if review.reason_tags else "nominal"
        if primary_reason == "advisor_disabled":
            return None
        message = template.format(reason=primary_reason.replace("_", " "))

        self._last_ts = now
        return message
