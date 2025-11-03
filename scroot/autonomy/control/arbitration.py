"""Advisor arbitration pipeline implementation."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace
from typing import Iterable, Optional, Tuple

from autonomy.control.config import AdvisorRuntimeConfig, ContextConfig, NavigationIntentConfig
from autonomy.utils.data_structures import (
    ActuatorCommand,
    AdvisorReview,
    AdvisorVerdict,
    ContextSnapshot,
    LaneType,
    NavigationDecision,
    NavigationSubGoal,
    SafetyCaps,
    VehicleEnvelope,
)


def _fail_stop() -> ActuatorCommand:
    return ActuatorCommand(steer=0.0, throttle=0.0, brake=1.0)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class RuleBasedAdvisor:
    """Deterministic advisor producing ALLOW/AMEND/BLOCK directives."""

    def __init__(
        self,
        config: AdvisorRuntimeConfig,
        max_vehicle_speed_mps: float,
        vehicle: Optional[VehicleEnvelope] = None,
    ) -> None:
        self.config = config
        self._max_vehicle_speed = max_vehicle_speed_mps
        self._vehicle = vehicle

    def evaluate(
        self,
        timestamp: float,
        perception_objects: Iterable,
        decision: NavigationDecision,
        proposed: ActuatorCommand,
        context: ContextSnapshot,
        caps: SafetyCaps,
        has_goal: bool,
    ) -> AdvisorReview:
        start = time.perf_counter()
        reason_tags: list[str] = []
        verdict = AdvisorVerdict.ALLOW
        amended: Optional[ActuatorCommand] = None

        hazard = decision.hazard_level
        meta = decision.metadata
        nearest_distance = float(meta.get("nearest_obstacle_distance_m", math.inf))
        lane_conf = context.confidence
        sensor_conf = context.sensor_confidence
        desired_speed = decision.desired_speed
        required_lateral = self._vehicle.required_lateral_clearance() if self._vehicle else 0.0
        clearance_left = float(meta.get("clearance_left_m", math.inf))
        clearance_right = float(meta.get("clearance_right_m", math.inf))
        lane_width = float(meta.get("lane_width_m", 0.0))

        detection_counts: dict[str, int] = {}
        for obj in perception_objects:
            label = getattr(obj, "label", "").lower()
            if label:
                detection_counts[label] = detection_counts.get(label, 0) + 1

        ttc = meta.get("time_to_collision_s")
        if ttc is None and math.isfinite(nearest_distance) and desired_speed > 0.05:
            ttc = nearest_distance / max(desired_speed, 1e-3)

        def _speed_to_throttle(speed_mps: float) -> float:
            return _clamp(speed_mps / max(self._max_vehicle_speed, 1e-3), 0.0, 1.0)

        # Collision blocks -------------------------------------------------
        if ttc is not None and ttc <= self.config.ttc_block_s:
            verdict = AdvisorVerdict.BLOCK
            reason_tags.append("ttc_low")

        if hazard >= 0.9:
            verdict = AdvisorVerdict.BLOCK
            reason_tags.append("hazard_peak")

        if sensor_conf < 0.3 and desired_speed > 0.5:
            verdict = AdvisorVerdict.BLOCK
            reason_tags.append("sensor_degradation")

        if self._vehicle:
            required_forward = self._vehicle.required_forward_clearance()
            if nearest_distance < required_forward:
                verdict = AdvisorVerdict.BLOCK
                reason_tags.append("forward_clearance")

            if lane_width and lane_width < self._vehicle.width_m + 2.0 * self._vehicle.clearance_margin_m:
                verdict = AdvisorVerdict.BLOCK
                reason_tags.append("lane_too_narrow")

            min_side_clearance = min(clearance_left, clearance_right)
            if min_side_clearance < required_lateral:
                verdict = AdvisorVerdict.BLOCK
                reason_tags.append("envelope_violation")

        if detection_counts.get("person", 0) > 0 and desired_speed > 2.5 and verdict != AdvisorVerdict.BLOCK:
            verdict = AdvisorVerdict.AMEND
            amended = ActuatorCommand(
                steer=proposed.steer,
                throttle=min(proposed.throttle, _speed_to_throttle(self.config.amend_speed_cap_mps)),
                brake=max(proposed.brake, 0.35),
            )
            reason_tags.append("pedestrian_close")

        if context.lane_type == LaneType.SIDEWALK and meta.get("intended_context") == "road":
            verdict = AdvisorVerdict.BLOCK
            reason_tags.append("lane_mismatch")

        if verdict == AdvisorVerdict.BLOCK and not reason_tags:
            reason_tags.append("blocking")

        # Amendments -------------------------------------------------------
        if verdict != AdvisorVerdict.BLOCK:
            amended = None
            # Speed caps based on mindset/context
            if caps.active and desired_speed > caps.max_speed_mps + 1e-3:
                target_speed = min(caps.max_speed_mps, self.config.amend_speed_cap_mps)
                amended = ActuatorCommand(
                    steer=proposed.steer,
                    throttle=_speed_to_throttle(target_speed),
                    brake=max(proposed.brake, 0.1),
                )
                verdict = AdvisorVerdict.AMEND
                reason_tags.append("cap_speed")

            vulnerable_distance = float(meta.get("nearest_vru_distance_m", math.inf))
            if (
                verdict != AdvisorVerdict.BLOCK
                and vulnerable_distance < math.inf
                and vulnerable_distance < max(2.5, desired_speed * 1.2)
            ):
                amended = ActuatorCommand(
                    steer=proposed.steer,
                    throttle=min(proposed.throttle, _speed_to_throttle(self.config.amend_speed_cap_mps)),
                    brake=max(proposed.brake, 0.25),
                )
                verdict = AdvisorVerdict.AMEND
                reason_tags.append("vru_slow")

            if (
                verdict != AdvisorVerdict.BLOCK
                and lane_conf < self.config.min_conf_for_allow
                and desired_speed > 0.4
            ):
                if self.config.mode == "strict":
                    verdict = AdvisorVerdict.BLOCK
                    amended = None
                    reason_tags.append("unknown_lane")
                else:
                    amended = ActuatorCommand(
                        steer=proposed.steer,
                        throttle=min(proposed.throttle, _speed_to_throttle(self.config.amend_speed_cap_mps)),
                        brake=max(proposed.brake, 0.2),
                    )
                    verdict = AdvisorVerdict.AMEND
                    reason_tags.append("unknown_lane")

            bike_lane_offset = meta.get("bike_lane_offset")
            if (
                verdict != AdvisorVerdict.BLOCK
                and bike_lane_offset is not None
                and float(bike_lane_offset) > 0.1
            ):
                amended = ActuatorCommand(
                    steer=_clamp(proposed.steer + 0.3, -1.0, 1.0),
                    throttle=min(proposed.throttle, _speed_to_throttle(self.config.amend_speed_cap_mps)),
                    brake=proposed.brake,
                )
                verdict = AdvisorVerdict.AMEND
                reason_tags.append("lane_bias_right")

            if (
                verdict != AdvisorVerdict.BLOCK
                and not has_goal
                and caps.active
                and caps.max_speed_mps < 1.0
            ):
                amended = ActuatorCommand(
                    steer=proposed.steer,
                    throttle=min(proposed.throttle, _speed_to_throttle(1.0)),
                    brake=max(proposed.brake, 0.3),
                )
                verdict = AdvisorVerdict.AMEND
                reason_tags.append("ambient_crawl")

            if self._vehicle:
                min_side_clearance = min(clearance_left, clearance_right)
                if min_side_clearance < required_lateral + 0.15:
                    steer_bias = -0.2 if clearance_left < clearance_right else 0.2
                    amended = ActuatorCommand(
                        steer=_clamp(proposed.steer + steer_bias, -1.0, 1.0),
                        throttle=min(
                            proposed.throttle,
                            _speed_to_throttle(self.config.amend_speed_cap_mps * 0.6),
                        ),
                        brake=max(proposed.brake, 0.25),
                    )
                    verdict = AdvisorVerdict.AMEND
                    reason_tags.append("side_clearance_bias")

        if not reason_tags:
            reason_tags.append("nominal")

        latency_ms = (time.perf_counter() - start) * 1000.0
        safe_to_release = verdict != AdvisorVerdict.BLOCK and hazard < 0.35 and (ttc is None or ttc > self.config.ttc_block_s * 1.5)

        return AdvisorReview(
            verdict=verdict,
            reason_tags=tuple(dict.fromkeys(reason_tags)),
            latency_ms=latency_ms,
            timestamp=timestamp,
            amended_command=amended,
            safe_to_release=safe_to_release,
        )


@dataclass(slots=True)
class NavigationIntentManager:
    """Tracks temporary navigation sub-goals."""

    config: NavigationIntentConfig
    _active_goal: Optional[str] = None
    _goal_start: float = 0.0

    def update(
        self,
        timestamp: float,
        context: ContextSnapshot,
        subgoal_hint: Optional[str],
    ) -> NavigationSubGoal:
        if not self.config.prefer_safer_lane_first or subgoal_hint is None:
            self._active_goal = None
        else:
            if self._active_goal is None:
                self._active_goal = f"merge_{subgoal_hint}"
                self._goal_start = timestamp
            elif timestamp - self._goal_start > self.config.max_lane_change_time_s:
                self._active_goal = None

        status = "idle"
        goal_type = "none"
        if self._active_goal:
            status = "active"
            goal_type = self._active_goal
            if context.lane_type == LaneType.BIKE_LANE:
                status = "complete"
                self._active_goal = None

        return NavigationSubGoal(goal_type=goal_type, status=status)


class SafetyGate:
    """Final gate that clamps actuator commands and enforces fail-safe stops."""

    def __init__(
        self,
        max_vehicle_speed: float,
        context_config: ContextConfig,
        intent_config: NavigationIntentConfig,
        vehicle: Optional[VehicleEnvelope] = None,
    ) -> None:
        self._max_vehicle_speed = max_vehicle_speed
        self._context_config = context_config
        self._intent_config = intent_config
        self._vehicle = vehicle

    def apply(
        self,
        base_command: ActuatorCommand,
        decision: NavigationDecision,
        context: ContextSnapshot,
        caps: SafetyCaps,
        block_active: bool,
        ambient_mode: bool,
        has_goal: bool,
    ) -> Tuple[ActuatorCommand, Tuple[str, ...]]:
        tags: list[str] = []

        if block_active:
            tags.append("advisor_block")
            return _fail_stop(), tuple(tags)

        steer, throttle, brake = base_command.steer, base_command.throttle, base_command.brake

        if context.sensor_confidence < 0.25:
            tags.append("sensor_drop")
            return _fail_stop(), tuple(tags)

        if caps.active:
            ratio = caps.max_speed_mps / max(self._max_vehicle_speed, 1e-3)
            ratio = _clamp(ratio, 0.0, 1.0)
            if throttle > ratio:
                throttle = ratio
                tags.append("caps_speed")

        nearest_distance = decision.metadata.get("nearest_obstacle_distance_m")
        if nearest_distance is None:
            nearest_distance = context.metadata.get("nearest_obstacle_distance_m")
        min_forward_clearance = caps.min_clearance_m
        if self._vehicle:
            min_forward_clearance = max(min_forward_clearance, self._vehicle.required_forward_clearance())
        if nearest_distance is not None and nearest_distance < min_forward_clearance:
            throttle = 0.0
            brake = max(brake, 0.6)
            tags.append("clearance_hold")

        if self._vehicle:
            left_clearance = context.metadata.get("clearance_left_m", float("inf"))
            right_clearance = context.metadata.get("clearance_right_m", float("inf"))
            required_lateral = self._vehicle.required_lateral_clearance()
            min_side = min(left_clearance, right_clearance)
            if min_side < required_lateral:
                throttle = 0.0
                brake = max(brake, 0.5)
                tags.append("envelope_stop")
            elif min_side < required_lateral + 0.2:
                throttle = min(throttle, 0.3)
                tags.append("envelope_slow")

        if ambient_mode and not has_goal:
            throttle = min(throttle, 0.35)
            if context.confidence < self._context_config.min_confidence:
                throttle = 0.0
                brake = max(brake, 0.4)
                tags.append("ambient_uncertain")

        steer = _clamp(steer, -1.0, 1.0)
        throttle = _clamp(throttle, 0.0, 1.0)
        brake = _clamp(brake, 0.0, 1.0)

        if brake > 0.05:
            throttle = min(throttle, 0.1)

        return ActuatorCommand(steer=steer, throttle=throttle, brake=brake), tuple(dict.fromkeys(tags))


@dataclass
class ArbitrationResult:
    review: AdvisorReview
    final_command: ActuatorCommand
    gate_tags: Tuple[str, ...]
    subgoal: NavigationSubGoal


class ControlArbiter:
    """Coordinates advisor, safety gate, and logging semantics."""

    def __init__(
        self,
        advisor_config: AdvisorRuntimeConfig,
        context_config: ContextConfig,
        intent_config: NavigationIntentConfig,
        max_vehicle_speed: float,
        vehicle: Optional[VehicleEnvelope] = None,
        enabled: bool = True,
    ) -> None:
        self._advisor = RuleBasedAdvisor(advisor_config, max_vehicle_speed, vehicle)
        self._gate = SafetyGate(max_vehicle_speed, context_config, intent_config, vehicle)
        self._intent = NavigationIntentManager(intent_config)
        self._advisor_config = advisor_config
        self._enabled = enabled
        self._last_review: Optional[AdvisorReview] = None
        self._timeout_streak = 0
        self._block_active = False
        self._block_release_candidate: Optional[float] = None

    def step(
        self,
        timestamp: float,
        perception_objects: Iterable,
        decision: NavigationDecision,
        proposed: ActuatorCommand,
        context: ContextSnapshot,
        caps: SafetyCaps,
        has_goal: bool,
        ambient_mode: bool,
        subgoal_hint: Optional[str],
    ) -> ArbitrationResult:
        if not self._enabled:
            review = AdvisorReview(
                verdict=AdvisorVerdict.ALLOW,
                reason_tags=("advisor_disabled",),
                latency_ms=0.0,
                timestamp=timestamp,
                amended_command=None,
                safe_to_release=True,
            )
            final_command, gate_tags = self._gate.apply(
                proposed,
                decision,
                context,
                caps,
                block_active=False,
                ambient_mode=ambient_mode,
                has_goal=has_goal,
            )
            subgoal = self._intent.update(timestamp, context, subgoal_hint)
            self._last_review = review
            return ArbitrationResult(
                review=review,
                final_command=final_command,
                gate_tags=gate_tags,
                subgoal=subgoal,
            )

        review = self._advisor.evaluate(
            timestamp,
            perception_objects,
            decision,
            proposed,
            context,
            caps,
            has_goal,
        )

        if review.latency_ms > self._advisor_config.evaluation_budget_ms:
            self._timeout_streak += 1
            if self._timeout_streak <= max(0, self._advisor_config.timeout_grace_ticks) and self._last_review:
                review = replace(
                    self._last_review,
                    latency_ms=review.latency_ms,
                    timestamp=timestamp,
                    reason_tags=tuple(dict.fromkeys(self._last_review.reason_tags + ("timeout",))),
                )
            else:
                review = AdvisorReview(
                    verdict=AdvisorVerdict.BLOCK,
                    reason_tags=("timeout",),
                    latency_ms=review.latency_ms,
                    timestamp=timestamp,
                    amended_command=None,
                    safe_to_release=False,
                )
        else:
            self._timeout_streak = 0

        if review.verdict == AdvisorVerdict.BLOCK:
            self._block_active = True
            self._block_release_candidate = None
        elif self._block_active:
            if review.safe_to_release:
                if self._block_release_candidate is None:
                    self._block_release_candidate = timestamp
                elif (timestamp - self._block_release_candidate) * 1000.0 >= self._advisor_config.block_debounce_ms:
                    self._block_active = False
                    self._block_release_candidate = None
            else:
                review = replace(
                    review,
                    verdict=AdvisorVerdict.BLOCK,
                    amended_command=None,
                    reason_tags=tuple(dict.fromkeys(review.reason_tags + ("block_hold",))),
                    safe_to_release=False,
                )

        self._last_review = review

        base_command = proposed
        if review.verdict == AdvisorVerdict.AMEND and review.amended_command is not None:
            base_command = review.amended_command
        if review.verdict == AdvisorVerdict.BLOCK:
            base_command = _fail_stop()

        final_command, gate_tags = self._gate.apply(
            base_command,
            decision,
            context,
            caps,
            block_active=self._block_active or review.verdict == AdvisorVerdict.BLOCK,
            ambient_mode=ambient_mode,
            has_goal=has_goal,
        )

        subgoal = self._intent.update(timestamp, context, subgoal_hint)

        return ArbitrationResult(
            review=review,
            final_command=final_command,
            gate_tags=gate_tags,
            subgoal=subgoal,
        )
