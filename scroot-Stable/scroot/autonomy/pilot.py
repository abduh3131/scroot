from __future__ import annotations

import json
import logging
import signal
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Callable, Iterator, Optional, Union

import cv2
import numpy as np

from autonomy.ai.command_interface import CommandInterface
from autonomy.control.arbitration import ControlArbiter
from autonomy.control.companion import RidingCompanion
from autonomy.control.config import (
    ArbiterRuntimeConfig,
    ContextConfig,
    NavigationIntentConfig,
    SafetyMindsetConfig,
)
from autonomy.control.context import ContextAnalyzer
from autonomy.control.controller import Controller, ControllerConfig
from autonomy.control.mindset import SafetyMindset
from autonomy.control.telemetry import TelemetryLogger
from autonomy.perception.lane_detection import LaneDetector, LaneDetectorConfig
from autonomy.perception.object_detection import ObjectDetector, ObjectDetectorConfig
from autonomy.planning.navigator import Navigator, NavigatorConfig
from autonomy.sensors.camera import CameraSensor
from autonomy.sensors.lidar import LidarSensor
from autonomy.utils.data_structures import (
    ActuatorCommand,
    ArbiterReview,
    ArbiterVerdict,
    ContextSnapshot,
    HighLevelCommand,
    LaneEstimate,
    NavigationDecision,
    NavigationSubGoal,
    PerceptionSummary,
    PilotTickData,
    SafetyCaps,
    VehicleEnvelope,
)


def _runtime_input_dir() -> Path:
    return Path.home() / "scroot" / "runtime_inputs"


def _default_camera_source() -> Path:
    return _runtime_input_dir() / "camera.jpg"


def _default_lidar_source() -> Path:
    return _runtime_input_dir() / "lidar.npy"


@dataclass
class PilotConfig:
    camera_source: Union[int, str, Path] = field(default_factory=_default_camera_source)
    camera_width: int = 1280
    camera_height: int = 720
    camera_fps: int = 30
    lidar_source: Path = field(default_factory=_default_lidar_source)
    model_name: str = "yolov8n.pt"
    confidence_threshold: float = 0.3
    iou_threshold: float = 0.4
    visualize: bool = False
    log_dir: Path = Path("logs")
    command_state_path: Optional[Path] = Path("logs/command_state.json")
    initial_command: Optional[str] = None
    command_file: Optional[Path] = None
    arbiter: ArbiterRuntimeConfig = field(default_factory=ArbiterRuntimeConfig)
    context: ContextConfig = field(default_factory=ContextConfig)
    navigation_intent: NavigationIntentConfig = field(default_factory=NavigationIntentConfig)
    safety_mindset: SafetyMindsetConfig = field(default_factory=SafetyMindsetConfig)
    safety_mindset_enabled: bool = False
    companion_persona: str = "calm_safe"
    lane_detector: LaneDetectorConfig = field(default_factory=LaneDetectorConfig)
    detector_device: str = "auto"
    vehicle_description: str = "Scooter"
    vehicle_width_m: float = 0.65
    vehicle_length_m: float = 1.2
    vehicle_height_m: float = 1.2
    vehicle_clearance_margin_m: float = 0.2
    calibration_reference_distance_m: float = 2.0
    calibration_reference_pixels: float = 220.0
    companion_enabled: bool = True


class AutonomyPilot:
    """Main orchestrator tying together sensing, perception, planning, and control."""

    def __init__(
        self,
        config: Optional[PilotConfig] = None,
        tick_callback: Optional[Callable[[PilotTickData], None]] = None,
    ) -> None:
        self.config = config or PilotConfig()
        self._tick_callback = tick_callback
        self._camera = CameraSensor(
            source=self.config.camera_source,
            width=self.config.camera_width,
            height=self.config.camera_height,
            fps=self.config.camera_fps,
        )
        self._lidar = LidarSensor(self.config.lidar_source)
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
                device=self.config.detector_device,
            )
        )
        self._navigator = Navigator(NavigatorConfig(), vehicle=self._vehicle)
        self._controller_config = ControllerConfig()
        self._controller = Controller(self._controller_config)
        self._lane_detector = LaneDetector(vehicle=self._vehicle, config=self.config.lane_detector)

        self._command_interface = CommandInterface(
            command_file=self.config.command_file,
            initial_command=self.config.initial_command,
        )
        self._running = False
        self._latest_decision: Optional[NavigationDecision] = None
        self._latest_review: Optional[ArbiterReview] = None
        self._latest_context: Optional[ContextSnapshot] = None
        self._latest_caps: Optional[SafetyCaps] = None
        self._latest_subgoal: Optional[NavigationSubGoal] = None
        self._latest_companion: Optional[str] = None
        self._latest_lane: Optional[LaneEstimate] = None
        self._latest_lidar: Optional[np.ndarray] = None

        if self.config.visualize or self.config.command_state_path:
            self.config.log_dir.mkdir(parents=True, exist_ok=True)

        # Safety mindset toggle
        if self.config.safety_mindset_enabled:
            self.config.safety_mindset.enabled = True

        self._context_analyzer = ContextAnalyzer(self.config.context, self.config.navigation_intent)
        self._mindset = SafetyMindset(self.config.safety_mindset, vehicle=self._vehicle)
        self._arbiter = ControlArbiter(
            arbiter_config=self.config.arbiter,
            context_config=self.config.context,
            intent_config=self.config.navigation_intent,
            max_vehicle_speed=self._controller_config.max_speed_mps,
            vehicle=self._vehicle,
        )
        self._telemetry = TelemetryLogger(self.config.log_dir)
        self._companion = (
            RidingCompanion(persona=self.config.companion_persona)
            if self.config.companion_enabled
            else None
        )

    @property
    def latest_decision(self) -> Optional[NavigationDecision]:
        return self._latest_decision

    @property
    def latest_review(self) -> Optional[ArbiterReview]:
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

    @property
    def latest_lane(self) -> Optional[LaneEstimate]:
        return self._latest_lane

    @property
    def latest_lidar(self) -> Optional[np.ndarray]:
        return self._latest_lidar

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

            lidar_success, lidar_ranges = self._lidar.read()
            if lidar_success and lidar_ranges is not None:
                self._latest_lidar = lidar_ranges

            lane_estimate = self._lane_detector.analyze(frame)
            self._latest_lane = lane_estimate

            perception_summary = self._detector.detect(frame)
            decision = self._navigator.plan(
                perception_summary.objects,
                perception_summary.frame_size,
                current_command,
                lane_estimate=None,
            )

            if current_command and current_command.command_type == "stop":
                decision = replace(decision, desired_speed=0.0, hazard_level=1.0, enforced_stop=True)

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
            companion_message = ""
            if self._companion and review:
                narrated = self._companion.narrate(review)
                if narrated:
                    companion_message = narrated

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
            self._latest_review = review
            self._latest_context = scene_context.snapshot
            self._latest_caps = caps
            self._latest_subgoal = arbitration.subgoal
            self._latest_companion = companion_message or None

            overlay_frame = None
            if self._tick_callback or self.config.visualize:
                overlay_frame = self._compose_overlay(
                    frame,
                    perception_summary,
                    lane_estimate,
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
                )

            if self.config.visualize and overlay_frame is not None:
                self._visualize(overlay_frame)

            self._export_command_state(current_command)

            yield command

        self._camera.close()
        logging.info("Pilot loop terminated")

    def stop(self) -> None:
        self._running = False

    def _export_command_state(
        self,
        current_command: Optional[HighLevelCommand],
    ) -> None:
        if self._command_interface and self.config.command_state_path:
            self._command_interface.export_state(self.config.command_state_path)

    def _compose_overlay(
        self,
        frame: np.ndarray,
        perception: PerceptionSummary,
        lane_estimate: Optional[LaneEstimate],
    ) -> np.ndarray:
        """Render perception overlays while keeping the raw camera feed untouched."""

        overlay = frame.copy()
        overlay = self._detector.draw_detections(overlay, perception.objects)

        if not lane_estimate:
            return overlay

        lane_layer = np.zeros_like(overlay)

        if lane_estimate.lane_area:
            cv2.fillPoly(lane_layer, [np.array(lane_estimate.lane_area)], (40, 200, 220))
            overlay = cv2.addWeighted(overlay, 1.0, lane_layer, 0.35, 0)

        if lane_estimate.points_left:
            cv2.polylines(overlay, [np.array(lane_estimate.points_left)], False, (255, 215, 0), 2)
        if lane_estimate.points_right:
            cv2.polylines(overlay, [np.array(lane_estimate.points_right)], False, (0, 191, 255), 2)

        if lane_estimate.points_left and lane_estimate.points_right:
            center_path = []
            for left_pt, right_pt in zip(lane_estimate.points_left, lane_estimate.points_right):
                cx = int((left_pt[0] + right_pt[0]) / 2)
                cy = int((left_pt[1] + right_pt[1]) / 2)
                center_path.append((cx, cy))
            if center_path:
                cv2.polylines(overlay, [np.array(center_path)], False, (102, 255, 102), 2)

        if lane_estimate.birdseye_outline:
            height, width = overlay.shape[:2]
            inset_height = max(80, height // 5)
            inset_width = max(80, width // 6)
            inset = np.zeros((inset_height, inset_width, 3), dtype=np.uint8)
            bev_array = np.array(lane_estimate.birdseye_outline, dtype=np.float32)
            max_x = max(1.0, float(np.max(bev_array[:, 0])))
            max_y = max(1.0, float(np.max(bev_array[:, 1])))
            scale_x = inset_width / max_x
            scale_y = inset_height / max_y
            bev_scaled = np.array(
                [
                    (
                        int(pt[0] * scale_x),
                        int(inset_height - 1 - pt[1] * scale_y),
                    )
                    for pt in lane_estimate.birdseye_outline
                ]
            )
            if bev_scaled.size:
                cv2.fillPoly(inset, [bev_scaled], (0, 120, 180))
                cv2.polylines(inset, [bev_scaled], True, (0, 255, 180), 1)
                inset_x = width - inset_width - 12
                inset_y = 12
                roi = overlay[inset_y : inset_y + inset_height, inset_x : inset_x + inset_width]
                blended = cv2.addWeighted(roi, 0.65, inset, 0.35, 0)
                overlay[inset_y : inset_y + inset_height, inset_x : inset_x + inset_width] = blended

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
        review: ArbiterReview,
        decision: NavigationDecision,
        context: ContextSnapshot,
        caps: SafetyCaps,
        gate_tags: tuple[str, ...],
        companion: Optional[str],
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
            goal_context = decision.goal_context if decision else ""
            lane_conf = ""
            if decision:
                meta = decision.metadata
                conf = meta.get("lane_confidence") if isinstance(meta, dict) else None
                offset = meta.get("lane_offset_m") if isinstance(meta, dict) else None
                if isinstance(conf, (float, int)) and isinstance(offset, (float, int)):
                    lane_conf = f" lane={float(conf):.2f} offset={float(offset):+.2f}m"
            print(
                " ".join(
                    [
                        f"time={elapsed:.2f}s",
                        f"steer={command.steer:+.3f}",
                        f"throttle={command.throttle:.3f}",
                        f"brake={command.brake:.3f}",
                        f"goal=\"{goal_context}\"",
                    ]
                )
                + lane_conf,
                flush=True,
            )

    logging.info("Autonomy run complete")


def parse_args(argv: Optional[list[str]] = None) -> PilotConfig:
    import argparse

    parser = argparse.ArgumentParser(description="Autonomous scooter pilot")
    parser.add_argument(
        "--camera",
        default=str(_default_camera_source()),
        help="Camera source index or path (defaults to runtime_inputs/camera.jpg)",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--confidence", type=float, default=0.3)
    parser.add_argument("--iou", type=float, default=0.4)
    parser.add_argument(
        "--device",
        choices=["auto", "cpu", "cuda", "quadro_p520"],
        default="auto",
        help=(
            "Choose inference acceleration: auto-detect, force CPU, force CUDA, or use the Quadro P520 compatibility mode."
        ),
    )
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--command", default=None, help="Initial natural language command")
    parser.add_argument("--command-file", default=None, help="Path to a file containing operator commands")
    parser.add_argument("--command-state", default="logs/command_state.json", help="Where to export parsed commands")
    parser.add_argument(
        "--lane-sensitivity",
        choices=["precision", "balanced", "aggressive"],
        default="balanced",
        help="Tune the lane detector between noise rejection and faster reactions",
    )
    parser.add_argument("--safety-mindset", choices=["on", "off"], default="off")
    parser.add_argument("--ambient", choices=["on", "off"], default="on")
    parser.add_argument(
        "--advisor",
        choices=["on", "off"],
        default="on",
        help="Enable the riding companion advisor narration",
    )
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
    command_state_path = Path(args.command_state).expanduser() if args.command_state else None

    context_settings = ContextConfig()
    navigation_intent = NavigationIntentConfig()
    navigation_intent.ambient_mode = args.ambient == "on"

    safety_mindset = SafetyMindsetConfig()
    safety_mindset.enabled = args.safety_mindset == "on"

    lane_detector = LaneDetectorConfig()
    if args.lane_sensitivity == "precision":
        lane_detector.min_confidence = 0.45
        lane_detector.smoothing = 0.8
    elif args.lane_sensitivity == "aggressive":
        lane_detector.min_confidence = 0.25
        lane_detector.smoothing = 0.4

    lidar_path = Path(args.lidar).expanduser()

    return PilotConfig(
        camera_source=args.camera,
        camera_width=args.width,
        camera_height=args.height,
        camera_fps=args.fps,
        lidar_source=lidar_path,
        model_name=args.model,
        confidence_threshold=args.confidence,
        iou_threshold=args.iou,
        visualize=args.visualize,
        log_dir=Path(args.log_dir).expanduser(),
        command_state_path=command_state_path,
        initial_command=args.command,
        command_file=command_file,
        arbiter=ArbiterRuntimeConfig(),
        context=context_settings,
        navigation_intent=navigation_intent,
        safety_mindset=safety_mindset,
        safety_mindset_enabled=safety_mindset.enabled,
        companion_persona=args.persona,
        companion_enabled=args.advisor == "on",
        lane_detector=lane_detector,
        detector_device=args.device,
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
