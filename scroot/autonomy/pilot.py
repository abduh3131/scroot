from __future__ import annotations

import json
import logging
import signal
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Callable, Iterator, Optional

import cv2
import numpy as np

from autonomy.ai.advisor import AdvisorConfig, create_advisor
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
from autonomy.perception.lane_detection import LaneDetector
from autonomy.perception.object_detection import ObjectDetector, ObjectDetectorConfig
from autonomy.planning.gps import GPSRoutePlanner
from autonomy.planning.navigator import Navigator, NavigatorConfig
from autonomy.sensors.gps import GPSConfig, GPSModule
from autonomy.utils.data_structures import (
    ActuatorCommand,
    AdvisorDirective,
    AdvisorReview,
    AdvisorVerdict,
    ContextSnapshot,
    HighLevelCommand,
    LaneObservation,
    NavigationDecision,
    NavigationSubGoal,
    PerceptionSummary,
    PilotTickData,
    GPSFix,
    RoutePlan,
    SafetyCaps,
    VehicleEnvelope,
)

from camera_autopilot import CameraMode as AutoCameraMode
from camera_autopilot import open as open_camera_autopilot


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
    advisor_backend: str = "vlm"
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
    gps_enabled: bool = False
    gps_port: Optional[str] = None
    gps_baudrate: int = 9600


class AutonomyPilot:
    """Main orchestrator tying together sensing, perception, planning, and control."""

    def __init__(
        self,
        config: PilotConfig | None = None,
        tick_callback: Optional[Callable[[PilotTickData], None]] = None,
    ) -> None:
        self.config = config or PilotConfig()
        self._tick_callback = tick_callback
        preferred_nodes: list[str | int] | None = None
        if isinstance(self.config.camera_source, (str, int)):
            preferred_nodes = [self.config.camera_source]
        preferred_mode = AutoCameraMode(
            fourcc="MJPG",
            width=int(self.config.camera_width),
            height=int(self.config.camera_height),
            fps=int(self.config.camera_fps),
        )
        try:
            self._camera = open_camera_autopilot(
                preferred_nodes=preferred_nodes,
                preferred_mode=preferred_mode,
            )
        except RuntimeError as exc:
            logging.error("Camera bootstrap failed: %s", exc)
            raise
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
        self._lane_detector = LaneDetector()
        self._navigator = Navigator(NavigatorConfig(), vehicle=self._vehicle)
        self._controller_config = ControllerConfig()
        self._controller = Controller(self._controller_config)
        self._advisor = None
        if self.config.advisor_enabled:
            advisor_config = AdvisorConfig(
                backend=self.config.advisor_backend,
                image_model=self.config.advisor_image_model,
                language_model=self.config.advisor_language_model,
                device=self.config.advisor_device,
            )
            try:
                self._advisor = create_advisor(advisor_config)
            except ImportError as exc:
                logging.warning("Advisor backend unavailable (%s); falling back to lightweight mode", exc)
                fallback = AdvisorConfig(backend="lite")
                self._advisor = create_advisor(fallback)

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
        self._gps_planner = GPSRoutePlanner()
        self._gps_module: Optional[GPSModule] = None
        if self.config.gps_enabled:
            if self.config.gps_port:
                gps_config = GPSConfig(
                    port=self.config.gps_port,
                    baudrate=self.config.gps_baudrate,
                    enabled=True,
                )
                self._gps_module = GPSModule(gps_config)
            else:
                logging.warning(
                    "GPS was enabled but no serial port was provided; disabling GPS integration."
                )

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
        if self._gps_module:
            try:
                self._gps_module.start()
            except RuntimeError as exc:
                logging.warning("GPS module failed to start: %s", exc)
                self._gps_module = None
        try:
            self._camera.start()
        except RuntimeError as exc:
            logging.error("Camera failed to start: %s", exc)
            self._running = False
            return
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
            lane_observation = self._lane_detector.detect(frame)
            gps_fix = self._gps_module.latest_fix() if self._gps_module else None
            route_plan: Optional[RoutePlan] = None
            if current_command and current_command.command_type == "navigate_to" and current_command.target:
                label = self._gps_planner.destination_label()
                if label != current_command.target:
                    if not self._gps_planner.set_destination(current_command.target):
                        logging.warning("Unable to resolve destination '%s'", current_command.target)
                if gps_fix:
                    route_plan = self._gps_planner.plan(gps_fix)
            else:
                self._gps_planner.clear_destination()

            decision = self._navigator.plan(
                perception_summary.objects,
                perception_summary.frame_size,
                current_command,
                lane_observation,
                route_plan,
                gps_fix,
            )

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

            overlay_frame = None
            if self._tick_callback or self.config.visualize:
                overlay_frame = self._compose_overlay(
                    frame,
                    perception_summary,
                    decision,
                    command,
                    arbitration,
                    caps,
                    lane_observation,
                    route_plan,
                )

            if self._tick_callback and overlay_frame is not None:
                self._publish_tick(
                    timestamp=tick_time,
                    overlay=overlay_frame,
                    command=command,
                    review=review,
                    decision=decision,
                    context=scene_context.snapshot,
                    caps=caps,
                    gate_tags=arbitration.gate_tags,
                    companion=companion_message,
                    lane=lane_observation,
                )

            if self.config.visualize and overlay_frame is not None:
                self._visualize(overlay_frame)

            self._export_state(current_command, advisor_directive, decision)

            yield command

        self._camera.stop()
        logging.info("Pilot loop terminated")

    def stop(self) -> None:
        self._running = False
        if self._gps_module:
            self._gps_module.stop()

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

    def _compose_overlay(
        self,
        frame: np.ndarray,
        perception: PerceptionSummary,
        decision: NavigationDecision,
        command: ActuatorCommand,
        arbitration,
        caps: SafetyCaps,
        lane: Optional[LaneObservation] = None,
        route: Optional[RoutePlan] = None,
    ) -> np.ndarray:
        overlay = frame.copy()
        overlay = self._detector.draw_detections(overlay, perception.objects)
        height, width = overlay.shape[:2]
        base_y = height - 20
        path_length = max(40, int(height * 0.45))
        steer_offset = int(command.steer * width * 0.25)
        half_span = max(20, int(width * 0.12))
        top_y = max(20, base_y - path_length)
        center_x = width // 2

        if lane and (lane.left_points or lane.right_points):
            if lane.left_points:
                pts = np.array(lane.left_points, dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(overlay, [pts], False, (0, 200, 255), 2)
            if lane.right_points:
                pts = np.array(lane.right_points, dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(overlay, [pts], False, (255, 140, 0), 2)
            if lane.vanishing_point:
                cv2.circle(overlay, tuple(map(int, lane.vanishing_point)), 6, (255, 255, 0), -1)

        cv2.line(overlay, (center_x, base_y), (center_x + steer_offset, top_y), (255, 255, 0), 3)
        cv2.line(
            overlay,
            (center_x - half_span, base_y),
            (center_x - half_span + steer_offset, top_y),
            (0, 255, 0),
            2,
        )
        cv2.line(
            overlay,
            (center_x + half_span, base_y),
            (center_x + half_span + steer_offset, top_y),
            (0, 255, 0),
            2,
        )

        bar_height = max(50, int(height * 0.2))
        bar_width = max(14, int(width * 0.025))
        throttle_x = 20
        throttle_y_top = base_y - bar_height
        throttle_level = int(bar_height * max(0.0, min(1.0, command.throttle)))
        throttle_fill_top = base_y - throttle_level
        cv2.rectangle(overlay, (throttle_x, throttle_y_top), (throttle_x + bar_width, base_y), (70, 70, 70), 2)
        cv2.rectangle(
            overlay,
            (throttle_x + 2, throttle_fill_top),
            (throttle_x + bar_width - 2, base_y - 2),
            (40, 200, 40),
            -1,
        )
        cv2.putText(
            overlay,
            "TH",
            (throttle_x, throttle_y_top - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

        brake_x = throttle_x + bar_width + 16
        brake_y_top = base_y - bar_height
        brake_level = int(bar_height * max(0.0, min(1.0, command.brake)))
        brake_fill_top = base_y - brake_level
        cv2.rectangle(overlay, (brake_x, brake_y_top), (brake_x + bar_width, base_y), (70, 70, 70), 2)
        cv2.rectangle(
            overlay,
            (brake_x + 2, brake_fill_top),
            (brake_x + bar_width - 2, base_y - 2),
            (20, 20, 220),
            -1,
        )
        cv2.putText(
            overlay,
            "BR",
            (brake_x, brake_y_top - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

        review = getattr(arbitration, "review", None)
        if review:
            color_map = {
                AdvisorVerdict.ALLOW: (60, 160, 60),
                AdvisorVerdict.AMEND: (0, 165, 255),
                AdvisorVerdict.BLOCK: (0, 0, 255),
            }
            header = overlay.copy()
            cv2.rectangle(header, (0, 0), (width, 70), color_map.get(review.verdict, (90, 90, 90)), -1)
            overlay = cv2.addWeighted(header, 0.3, overlay, 0.7, 0)
            summary = f"{review.verdict.value}: {', '.join(review.reason_tags)}"
            cv2.putText(
                overlay,
                summary,
                (12, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                overlay,
                f"Latency {review.latency_ms:.1f} ms",
                (12, 62),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (240, 240, 240),
                1,
            )

        info_lines = [
            f"Steer {command.steer:+.2f}",
            f"Throttle {command.throttle:.2f} Brake {command.brake:.2f}",
            f"Desired {decision.desired_speed:.1f} m/s",
        ]
        if decision.directive:
            info_lines.append(f"Directive {decision.directive}")
        if decision.goal_context:
            info_lines.append(f"Goal {decision.goal_context}")
        if lane and lane.detected:
            info_lines.append(f"Lane offset {lane.offset_normalized:+.2f} conf {lane.confidence:.2f}")
        if route:
            info_lines.append(
                f"Route {route.distance_m/1000:.2f} km bearing {route.bearing_deg:.0f}°"
            )
            if route.eta_s is not None:
                info_lines.append(f"ETA {route.eta_s/60:.1f} min")
        if caps and caps.active:
            info_lines.append(
                f"Caps {caps.source}: ≤{caps.max_speed_mps:.1f}m/s, ≥{caps.min_clearance_m:.1f}m"
            )
        gate_tags = getattr(arbitration, "gate_tags", ())
        if gate_tags:
            info_lines.append("Gate " + ", ".join(gate_tags))

        for idx, text in enumerate(info_lines):
            cv2.putText(
                overlay,
                text,
                (throttle_x + 2 * bar_width + 40, 30 + idx * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

        return overlay

    def _visualize(self, overlay: np.ndarray) -> None:
        cv2.imshow("AutonomyPilot", overlay)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.stop()

    def _publish_tick(
        self,
        *,
        timestamp: float,
        overlay: np.ndarray,
        command: ActuatorCommand,
        review: AdvisorReview,
        decision: NavigationDecision,
        context: ContextSnapshot,
        caps: SafetyCaps,
        gate_tags: tuple[str, ...],
        companion: Optional[str],
        lane: Optional[LaneObservation],
    ) -> None:
        if not self._tick_callback:
            return
        try:
            packet = PilotTickData(
                timestamp=timestamp,
                overlay=overlay.copy(),
                command=command,
                review=review,
                decision=decision,
                context=context,
                caps=caps,
                gate_tags=gate_tags,
                companion=companion,
                lane=lane,
            )
            self._tick_callback(packet)
        except Exception:  # pragma: no cover - defensive logging
            logging.exception("Failed to publish tick data to GUI")

    def submit_command(self, text: str) -> Optional[HighLevelCommand]:
        if not self._command_interface:
            logging.warning("Command interface is disabled; ignoring: %s", text)
            return None
        command = self._command_interface.submit(text)
        logging.info("Operator command submitted: %s -> %s", text, command.command_type)
        return command


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
