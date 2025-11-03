"""Telemetry helpers for advisor arbitration."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

from autonomy.utils.data_structures import (
    ActuatorCommand,
    AdvisorReview,
    AdvisorVerdict,
    ContextSnapshot,
    NavigationDecision,
    NavigationSubGoal,
    SafetyCaps,
)


class TelemetryLogger:
    def __init__(self, log_dir: Path) -> None:
        self.log_dir = log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.telemetry_path = self.log_dir / "telemetry.jsonl"
        self.incidents_path = self.log_dir / "incidents.jsonl"
        self._finalized = False

    def _append_json(self, payload: Dict[str, Any]) -> None:
        if self._finalized:
            return
        with self.telemetry_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload) + "\n")

    def _serialize_command(self, command: ActuatorCommand) -> Dict[str, float]:
        return {"steer": command.steer, "throttle": command.throttle, "brake": command.brake}

    def record(
        self,
        timestamp: float,
        decision: NavigationDecision,
        proposed: ActuatorCommand,
        result: ActuatorCommand,
        review: AdvisorReview,
        caps: SafetyCaps,
        context: ContextSnapshot,
        gate_tags: Iterable[str],
        subgoal: NavigationSubGoal,
        companion_message: Optional[str],
    ) -> None:
        payload: Dict[str, Any] = {
            "timestamp": timestamp,
            "advisor": {
                "decision": review.verdict.value,
                "reason_tags": list(review.reason_tags),
                "latency_ms": review.latency_ms,
            },
            "navigation": {
                "desired_speed": decision.desired_speed,
                "hazard_level": decision.hazard_level,
                "metadata": dict(decision.metadata),
                "subgoal": {"type": subgoal.goal_type, "status": subgoal.status},
            },
            "proposed_cmd": self._serialize_command(proposed),
            "final_cmd": self._serialize_command(result),
            "caps": {
                "active": caps.active,
                "source": caps.source,
                "profile": caps.profile,
                "max_speed_mps": caps.max_speed_mps,
                "min_clearance_m": caps.min_clearance_m,
                "annotations": dict(caps.annotations),
            },
            "context": {
                "lane_type": context.lane_type.value,
                "confidence": context.confidence,
                "sensor_confidence": context.sensor_confidence,
                "metadata": dict(context.metadata),
            },
            "gate_tags": list(gate_tags),
        }

        if companion_message:
            payload["companion"] = companion_message

        self._append_json(payload)

        if review.verdict in {AdvisorVerdict.AMEND, AdvisorVerdict.BLOCK}:
            incident = {
                "timestamp": timestamp,
                "decision": review.verdict.value,
                "reason_tags": list(review.reason_tags),
                "clip_pre_s": 5,
                "clip_post_s": 5,
            }
            with self.incidents_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(incident) + "\n")

    def record_environment(self, info: Dict[str, Any]) -> None:
        payload = {"event": "environment", "data": info}
        self._append_json(payload)

    def record_shutdown(self, info: Dict[str, Any]) -> None:
        payload = {"event": "shutdown", "data": info}
        self._append_json(payload)

    def finalize(self) -> None:
        if self._finalized:
            return
        try:
            with self.telemetry_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps({"event": "telemetry_closed"}) + "\n")
        finally:
            self._finalized = True
