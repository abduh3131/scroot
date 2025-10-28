from __future__ import annotations

import json
import logging
import signal
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Iterator, Optional

import cv2
import numpy as np

from autonomy.ai.advisor import AdvisorConfig, SituationalAdvisor
from autonomy.ai.command_interface import CommandInterface
from autonomy.control.arbitration import ControlArbiter
from autonomy.control.companion import RidingCompanion
from autonomy.control.config import (
    AdvisorRuntimeConfig,
    ContextConfig,
    NavigationIntentConfig,
    SafetyMindsetConfig,
)
from autonomy.control.context import ContextAnalyzer
from autonomy.control.controller import Controller, ControllerConfig
from autonomy.control.mindset import SafetyMindset
from autonomy.control.telemetry import TelemetryLogger
from autonomy.perception.object_detection import ObjectDetector, ObjectDetectorConfig
from autonomy.planning.navigator import Navigator, NavigatorConfig
from autonomy.sensors.camera import CameraSensor
from autonomy.utils.data_structures import (
    ActuatorCommand,
    AdvisorDirective,
    AdvisorReview,
    ContextSnapshot,
    HighLevelCommand,
    NavigationDecision,
    NavigationSubGoal,
    PerceptionSummary,
    SafetyCaps,
    VehicleEnvelope,
)


@dataclass
class PilotConfig:
    camera_source: int | str = 0
    camera_width: int = 1280
    camera_height: int = 720
    camera_fps: int = 30
    model_name: str = "yolov8n.pt"
    confidence_threshold: float = 0.3
    iou_threshold: float = 0.4
    visualize: bool = False
    log_dir: Path = Path("logs")
    advisor_enabled: bool = True
    advisor_image_model: str = "Salesforce/blip-image-captioning-base"
    advisor_language_model: str = "google/flan-t5-small"
    advisor_device: Optional[str] = None
    advisor_state_path: Optional[Path] = Path("logs/advisor_state.json")
    command_state_path: Optional[Path] = Path("logs/command_state.json")
    initial_command: Optional[str] = None
    command_file: Optional[Path] = None
    advisor: AdvisorRuntimeConfig = field(default_factory=AdvisorRuntimeConfig)
    context: ContextConfig = field(default_factory=ContextConfig)
    navigation_intent: NavigationIntentConfig = field(default_factory=NavigationIntentConfig)
    safety_mindset: SafetyMindsetConfig = field(default_factory=SafetyMindsetConfig)
    safety_mindset_enabled: bool = False
    companion_persona: str = "calm_safe"
    vehicle_description: str = "Scooter"
    vehicle_width_m: float = 0.65
    vehicle_length_m: float = 1.2
    vehicle_height_m: float = 1.2
    vehicle_clearance_margin_m: float = 0.2
    calibration_reference_distance_m: float = 2.0
    calibration_reference_pixels: float = 220.0


class AutonomyPilot:
    """Main orchestrator tying together sensing, perception, planning, and control."""

    def __init__(self, config: PilotConfig | None = None) -> None:
        self.config = config or PilotConfig()
        self._camera = CameraSensor(
            source=self.config.camera_source,
            width=self.config.camera_width,
            height=self.config.camera_height,
            fps=self.config.camera_fps,
        )
        self._vehicle = VehicleEnvelope(
            width_m=self.config.vehicle_width_m,
            length_m=self.config.vehicle_length_m,
            height_m=self.config.vehicle_height_m,
            description=self.config.vehicle_description,
            clearance_margin_m=self.config.vehicle_clearance_margin_m,
            calibration_reference_distance_m=self.config.calibration_reference_distance_m,
            calibration_reference_pixels=self.config.calibration_reference_pixels,
        )

        self._detector = ObjectDetector(
            ObjectDetectorConfig(
                model_name=self.config.model_name,
                confidence_threshold=self.config.confidence_threshold,
                iou_threshold=self.config.iou_threshold,
            )
        )
        self._navigator = Navigator(NavigatorConfig(), vehicle=self._vehicle)
        self._controller_config = ControllerConfig()
        self._controller = Controller(self._controller_config)
        self._advisor = None
        if self.config.advisor_enabled:
            advisor_config = AdvisorConfig(
                image_model=self.config.advisor_image_model,
                language_model=self.config.advisor_language_model,
                device=self.config.advisor_device,
            )
            self._advisor = SituationalAdvisor(advisor_config)

        self._command_interface = CommandInterface(
            command_file=self.config.command_file,
            initial_command=self.config.initial_command,
        )
        self._running = False
        self._latest_decision: Optional[NavigationDecision] = None
        self._latest_directive: Optional[AdvisorDirective] = None
        self._latest_review: Optional[AdvisorReview] = None
        self._latest_context: Optional[ContextSnapshot] = None
        self._latest_caps: Optional[SafetyCaps] = None
        self._latest_subgoal: Optional[NavigationSubGoal] = None
        self._latest_companion: Optional[str] = None

        if self.config.visualize or self.config.advisor_state_path or self.config.command_state_path:
            self.config.log_dir.mkdir(parents=True, exist_ok=True)

        # Safety mindset toggle
        if self.config.safety_mindset_enabled:
            self.config.safety_mindset.enabled = True

        self._context_analyzer = ContextAnalyzer(self.config.context, self.config.navigation_intent)
        self._mindset = SafetyMindset(self.config.safety_mindset, vehicle=self._vehicle)
        self._arbiter = ControlArbiter(
            advisor_config=self.config.advisor,
            context_config=self.config.context,
            intent_config=self.config.navigation_intent,
            max_vehicle_speed=self._controller_config.max_speed_mps,
            vehicle=self._vehicle,
            enabled=self.config.advisor_enabled,
        )
        self._telemetry = TelemetryLogger(self.config.log_dir)
        self._companion = RidingCompanion(persona=self.config.companion_persona)

    @property
    def latest_decision(self) -> Optional[NavigationDecision]:
        return self._latest_decision

    @property
    def latest_directive(self) -> Optional[AdvisorDirective]:
        return self._latest_directive

    @property
    def latest_review(self) -> Optional[AdvisorReview]:
        return self._latest_review

    @property
    def latest_context(self) -> Optional[ContextSnapshot]:
        return self._latest_context

    @property
    def latest_caps(self) -> Optional[SafetyCaps]:
        return self._latest_caps

    @property
    def latest_subgoal(self) -> Optional[NavigationSubGoal]:
        return self._latest_subgoal

    @property
    def latest_companion(self) -> Optional[str]:
        return self._latest_companion

    def run(self) -> Iterator[ActuatorCommand]:
        logging.info("Starting autonomous pilot loop")
        self._running = True
        frame_iterator = self._camera.frames()
        for success, frame in frame_iterator:
            if not self._running:
                break
            if not success or frame is None:
                logging.warning("Failed to read frame from camera")
                continue

            current_command: Optional[HighLevelCommand] = None
            if self._command_interface:
                current_command = self._command_interface.poll()

            perception_summary = self._detector.detect(frame)
            decision = self._navigator.plan(perception_summary.objects, perception_summary.frame_size, current_command)

            if current_command and current_command.command_type == "stop":
                decision = replace(decision, desired_speed=0.0, hazard_level=1.0, enforced_stop=True)

            advisor_directive: Optional[AdvisorDirective] = None
            if self._advisor:
                advisor_directive = self._advisor.analyze(frame, perception_summary, decision, current_command)
                decision = replace(
                    decision,
                    enforced_stop=decision.enforced_stop or advisor_directive.enforced_stop,
                    directive=advisor_directive.directive,
                    caption=advisor_directive.caption,
                )
                if advisor_directive.enforced_stop:
                    decision = replace(decision, desired_speed=0.0, hazard_level=1.0)

            proposed_command = self._controller.command(decision)

            scene_context = self._context_analyzer.analyze(decision)
            caps = self._mindset.evaluate(scene_context.snapshot, decision)
            has_goal = bool(current_command and current_command.command_type != "stop")
            ambient_mode = self.config.navigation_intent.ambient_mode

            tick_time = time.time()
            arbitration = self._arbiter.step(
                timestamp=tick_time,
                perception_objects=perception_summary.objects,
                decision=decision,
                proposed=proposed_command,
                context=scene_context.snapshot,
                caps=caps,
                has_goal=has_goal,
                ambient_mode=ambient_mode,
                subgoal_hint=scene_context.subgoal_hint,
            )

            command = arbitration.final_command
            review = arbitration.review
            companion_message = self._companion.narrate(review)

            self._telemetry.record(
                timestamp=tick_time,
                decision=decision,
                proposed=proposed_command,
                result=command,
                review=review,
                caps=caps,
                context=scene_context.snapshot,
                gate_tags=arbitration.gate_tags,
                subgoal=arbitration.subgoal,
                companion_message=companion_message,
            )

            self._latest_decision = decision
            self._latest_directive = advisor_directive
            self._latest_review = review
            self._latest_context = scene_context.snapshot
            self._latest_caps = caps
            self._latest_subgoal = arbitration.subgoal
            self._latest_companion = companion_message

            if self.config.visualize:
                self._visualize(frame, perception_summary, decision, command)

            self._export_state(current_command, advisor_directive, decision)

            yield command

        self._camera.close()
        logging.info("Pilot loop terminated")

    def stop(self) -> None:
        self._running = False

    def _export_state(
        self,
        current_command: Optional[HighLevelCommand],
        advisor_directive: Optional[AdvisorDirective],
        decision: NavigationDecision,
    ) -> None:
        if self._command_interface and self.config.command_state_path:
            self._command_interface.export_state(self.config.command_state_path)

        if advisor_directive and self.config.advisor_state_path:
            payload = {
                "directive": advisor_directive.directive,
                "enforced_stop": advisor_directive.enforced_stop,
                "caption": advisor_directive.caption,
                "goal_context": decision.goal_context,
            }
            path = self.config.advisor_state_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def _visualize(
        self,
        frame: np.ndarray,
        perception: PerceptionSummary,
        decision: NavigationDecision,
        command: ActuatorCommand,
    ) -> None:
        visual = frame.copy()
        visual = self._detector.draw_detections(visual, perception.objects)
        text_lines = [
            f"steer={command.steer:+.2f}",
            f"throttle={command.throttle:.2f}",
            f"brake={command.brake:.2f}",
        ]
        if self._latest_review:
            text_lines.append(
                f"advisor={self._latest_review.verdict.value} ({','.join(self._latest_review.reason_tags)})"
            )
        if decision.directive:
            text_lines.append(f"advisor: {decision.directive}")
        text_lines.append(f"goal: {decision.goal_context}")
        for idx, text in enumerate(text_lines):
            cv2.putText(visual, text, (10, 30 + idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow("AutonomyPilot", visual)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.stop()


@contextmanager
def graceful_shutdown(pilot: AutonomyPilot):
    def handler(signum, frame):  # pragma: no cover - interacts with system signals
        logging.info("Received signal %s, shutting down", signum)
        pilot.stop()

    original = signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    try:
        yield
    finally:
        pilot.stop()
        signal.signal(signal.SIGINT, original)
        signal.signal(signal.SIGTERM, original)
        cv2.destroyAllWindows()


def run_pilot(config: PilotConfig) -> None:
    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s:%(name)s:%(message)s")
    pilot = AutonomyPilot(config)
    start_time = time.time()

    with graceful_shutdown(pilot):
        for command in pilot.run():
            elapsed = time.time() - start_time
            decision = pilot.latest_decision
            directive = decision.directive if decision else ""
            goal_context = decision.goal_context if decision else ""
            print(
                " ".join(
                    [
                        f"time={elapsed:.2f}s",
                        f"steer={command.steer:+.3f}",
                        f"throttle={command.throttle:.3f}",
                        f"brake={command.brake:.3f}",
                        f"directive=\"{directive}\"",
                        f"goal=\"{goal_context}\"",
                    ]
                ),
                flush=True,
            )

    logging.info("Autonomy run complete")


def parse_args(argv: Optional[list[str]] = None) -> PilotConfig:
    import argparse

    parser = argparse.ArgumentParser(description="Autonomous scooter pilot")
    parser.add_argument("--camera", default=0, help="Camera source index or path")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--confidence", type=float, default=0.3)
    parser.add_argument("--iou", type=float, default=0.4)
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--disable-advisor", action="store_true")
    parser.add_argument("--advisor-image-model", default="Salesforce/blip-image-captioning-base")
    parser.add_argument("--advisor-language-model", default="google/flan-t5-small")
    parser.add_argument("--advisor-device", default=None)
    parser.add_argument("--command", default=None, help="Initial natural language command")
    parser.add_argument("--command-file", default=None, help="Path to a file containing operator commands")
    parser.add_argument("--advisor-state", default="logs/advisor_state.json", help="Where to export advisor decisions")
    parser.add_argument("--command-state", default="logs/command_state.json", help="Where to export parsed commands")
    parser.add_argument("--advisor-mode", choices=["strict", "normal"], default="normal")
    parser.add_argument("--safety-mindset", choices=["on", "off"], default="off")
    parser.add_argument("--ambient", choices=["on", "off"], default="on")
    parser.add_argument(
        "--persona",
        choices=["calm_safe", "smart_scout", "playful"],
        default="calm_safe",
        help="Narration persona for the riding companion",
    )
    parser.add_argument("--vehicle-description", default="Scooter", help="Describe the platform for logs")
    parser.add_argument("--vehicle-width", type=float, default=0.65, help="Vehicle width in meters")
    parser.add_argument("--vehicle-length", type=float, default=1.2, help="Vehicle length in meters")
    parser.add_argument("--vehicle-height", type=float, default=1.2, help="Vehicle height in meters")
    parser.add_argument(
        "--clearance-margin",
        type=float,
        default=0.2,
        help="Extra lateral margin in meters on each side",
    )
    parser.add_argument(
        "--calibration-distance",
        type=float,
        default=2.0,
        help="Reference distance in meters used to approximate depth",
    )
    parser.add_argument(
        "--calibration-pixels",
        type=float,
        default=220.0,
        help="Pixel width of the vehicle at the calibration distance",
    )

    args = parser.parse_args(argv)

    if args.vehicle_width <= 0 or args.vehicle_length <= 0 or args.vehicle_height <= 0:
        parser.error("Vehicle dimensions must be greater than zero.")
    if args.clearance_margin < 0:
        parser.error("--clearance-margin cannot be negative")
    if args.calibration_distance <= 0 or args.calibration_pixels <= 0:
        parser.error("Calibration distance/pixels must be positive")

    command_file = Path(args.command_file).expanduser() if args.command_file else None
    advisor_state_path = Path(args.advisor_state).expanduser() if args.advisor_state else None
    command_state_path = Path(args.command_state).expanduser() if args.command_state else None

    advisor_settings = AdvisorRuntimeConfig()
    if args.advisor_mode == "strict":
        advisor_settings.mode = "strict"
        advisor_settings.min_conf_for_allow = 0.7
        advisor_settings.ttc_block_s = 1.5
        advisor_settings.block_debounce_ms = 1000
        advisor_settings.timeout_grace_ticks = 0
    else:
        advisor_settings.mode = "normal"

    context_settings = ContextConfig()
    navigation_intent = NavigationIntentConfig()
    navigation_intent.ambient_mode = args.ambient == "on"

    safety_mindset = SafetyMindsetConfig()
    safety_mindset.enabled = args.safety_mindset == "on"

    return PilotConfig(
        camera_source=args.camera,
        camera_width=args.width,
        camera_height=args.height,
        camera_fps=args.fps,
        model_name=args.model,
        confidence_threshold=args.confidence,
        iou_threshold=args.iou,
        visualize=args.visualize,
        log_dir=Path(args.log_dir).expanduser(),
        advisor_enabled=not args.disable_advisor,
        advisor_image_model=args.advisor_image_model,
        advisor_language_model=args.advisor_language_model,
        advisor_device=args.advisor_device,
        advisor_state_path=advisor_state_path,
        command_state_path=command_state_path,
        initial_command=args.command,
        command_file=command_file,
        advisor=advisor_settings,
        context=context_settings,
        navigation_intent=navigation_intent,
        safety_mindset=safety_mindset,
        safety_mindset_enabled=safety_mindset.enabled,
        companion_persona=args.persona,
        vehicle_description=args.vehicle_description,
        vehicle_width_m=args.vehicle_width,
        vehicle_length_m=args.vehicle_length,
        vehicle_height_m=args.vehicle_height,
        vehicle_clearance_margin_m=args.clearance_margin,
        calibration_reference_distance_m=args.calibration_distance,
        calibration_reference_pixels=args.calibration_pixels,
    )


def main(argv: Optional[list[str]] = None) -> None:
    config = parse_args(argv)
    run_pilot(config)


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    main(sys.argv[1:])
