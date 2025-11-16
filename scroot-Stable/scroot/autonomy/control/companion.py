"""Human-style narration that mirrors arbiter decisions."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional

from autonomy.utils.data_structures import ArbiterReview


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

    def narrate(self, review: ArbiterReview) -> Optional[str]:
        now = time.time()
        if now - self._last_ts < self.min_interval_s:
            return None

        tones = PERSONA_TONES.get(self.persona, PERSONA_TONES["calm_safe"])
        template = tones.get(review.verdict.value, tones["ALLOW"])

        primary_reason = review.reason_tags[0] if review.reason_tags else "nominal"
        message = template.format(reason=primary_reason.replace("_", " "))

        self._last_ts = now
        return message
