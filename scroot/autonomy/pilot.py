from __future__ import annotations

import csv
import json
import logging
import math
import signal
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass, replace, field
from pathlib import Path
from typing import Iterator, Optional, TextIO

import cv2
import numpy as np

from autonomy.ai.advisor import AdvisorConfig, SituationalAdvisor
from autonomy.ai.command_interface import CommandInterface
from autonomy.control.controller import Controller, ControllerConfig
from autonomy.perception.object_detection import ObjectDetector, ObjectDetectorConfig
from autonomy.planning.navigator import Navigator, NavigatorConfig
from autonomy.planning.localization import LocalizationModule, LocalizationState
from autonomy.planning.summon import SummonPlanner
from autonomy.sensors import SensorManager, SensorSample, SensorSpec
from autonomy.core.mode_manager import ModeManager, ModeManagerConfig, ModeContext
from autonomy.safety.guardian import Guardian, GuardianConfig, GuardianDecision
from autonomy.utils.data_structures import (
    ActuatorCommand,
    AdvisorDirective,
    HighLevelCommand,
    NavigationDecision,
    PerceptionSummary,
    VehicleMode,
)


@dataclass
class HardwareMapping:
    """Maps normalized actuator commands to MCU-friendly units."""

    steer_center: float = 1500.0  # microseconds for straight steering servo
    steer_range: float = 400.0  # +/- delta microseconds for steering extremes
    throttle_offset: float = 0.0  # baseline throttle output (e.g., PWM duty)
    throttle_scale: float = 255.0  # scale factor applied to throttle command
    brake_offset: float = 0.0  # baseline brake output
    brake_scale: float = 255.0  # scale factor applied to brake command


@dataclass
class PilotConfig:
    camera_source: int | str = 0
    camera_width: int = 1280
    camera_height: int = 720
    camera_fps: int = 30
    camera_probe_sources: list[object] = field(default_factory=list)
    camera_prefer_gstreamer: Optional[bool] = None
    camera_gstreamer_flip: int = 0
    camera_max_probe_sources: int = 6
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
    sensor_type: str = "camera"  # legacy single-sensor fields
    sensor_args: dict = field(default_factory=dict)
    sensors: list[SensorSpec] = field(default_factory=list)
    video_output_path: Optional[Path] = None
    command_log_path: Optional[Path] = None
    hardware_mapping: HardwareMapping = field(default_factory=HardwareMapping)
    mode_manager_config: ModeManagerConfig = field(default_factory=ModeManagerConfig)
    guardian_config: GuardianConfig = field(default_factory=GuardianConfig)
    initial_mode: VehicleMode = VehicleMode.CRUISE


class AutonomyPilot:
    """Main orchestrator tying together sensing, perception, planning, and control."""

    def __init__(self, config: PilotConfig | None = None) -> None:
        self.config = config or PilotConfig()
        self._sensor_specs: list[SensorSpec] = []
        self._lidar_names: list[str] = []
        self._sensor_manager = self._create_sensor_manager()
        self._detector = ObjectDetector(
            ObjectDetectorConfig(
                model_name=self.config.model_name,
                confidence_threshold=self.config.confidence_threshold,
                iou_threshold=self.config.iou_threshold,
            )
        )
        self._navigator = Navigator(NavigatorConfig())
        self._controller = Controller(ControllerConfig())
        self._mode_manager = ModeManager(self.config.mode_manager_config, initial_mode=self.config.initial_mode)
        self._guardian = Guardian(self.config.guardian_config)
        self._localization = LocalizationModule()
        self._summon_planner = SummonPlanner()
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
        self._latest_guardian: Optional[GuardianDecision] = None
        self._latest_mode_context: Optional[ModeContext] = None
        self._latest_localization: Optional[LocalizationState] = None
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._command_log_file: Optional[TextIO] = None
        self._command_log_writer: Optional[csv.writer] = None
        self._start_time: Optional[float] = None

        if self.config.visualize or self.config.advisor_state_path or self.config.command_state_path:
            self.config.log_dir.mkdir(parents=True, exist_ok=True)
        if self.config.video_output_path:
            self.config.video_output_path.parent.mkdir(parents=True, exist_ok=True)
        if self.config.command_log_path:
            self.config.command_log_path.parent.mkdir(parents=True, exist_ok=True)

    def _create_sensor_manager(self) -> SensorManager:
        specs = self._build_sensor_specs()
        primary_name = specs[0].name or specs[0].adapter
        self._sensor_specs = specs
        self._lidar_names = [
            (spec.name or spec.adapter)
            for spec in specs
            if spec.adapter.lower().startswith("lidar")
        ]
        logging.info(
            "Configured sensors: %s",
            ", ".join(f"{spec.name or spec.adapter}<{spec.adapter}>" for spec in specs),
        )
        return SensorManager(specs, primary_name=primary_name)

    def _build_sensor_specs(self) -> list[SensorSpec]:
        if self.config.sensors:
            return list(self.config.sensors)

        sensor_kwargs = dict(self.config.sensor_args)
        if self.config.sensor_type == "camera":
            sensor_kwargs.setdefault("source", self.config.camera_source)
            sensor_kwargs.setdefault("width", self.config.camera_width)
            sensor_kwargs.setdefault("height", self.config.camera_height)
            sensor_kwargs.setdefault("fps", self.config.camera_fps)
            if self.config.camera_probe_sources and "probe_sources" not in sensor_kwargs:
                sensor_kwargs["probe_sources"] = list(self.config.camera_probe_sources)
            if self.config.camera_prefer_gstreamer is not None and "prefer_gstreamer" not in sensor_kwargs:
                sensor_kwargs["prefer_gstreamer"] = self.config.camera_prefer_gstreamer
            if "gstreamer_flip_method" not in sensor_kwargs:
                sensor_kwargs["gstreamer_flip_method"] = self.config.camera_gstreamer_flip
            if "max_probe_sources" not in sensor_kwargs:
                sensor_kwargs["max_probe_sources"] = self.config.camera_max_probe_sources

        return [
            SensorSpec(
                adapter=self.config.sensor_type,
                name="primary",
                args=sensor_kwargs,
            )
        ]

    @property
    def latest_decision(self) -> Optional[NavigationDecision]:
        return self._latest_decision

    @property
    def latest_directive(self) -> Optional[AdvisorDirective]:
        return self._latest_directive

    @property
    def latest_guardian(self) -> Optional[GuardianDecision]:
        return self._latest_guardian

    @property
    def latest_mode_context(self) -> Optional[ModeContext]:
        return self._latest_mode_context

    @property
    def mode_manager(self) -> ModeManager:
        return self._mode_manager

    @property
    def sensors(self) -> SensorManager:
        return self._sensor_manager

    @property
    def latest_localization(self) -> Optional[LocalizationState]:
        return self._latest_localization

    def _collect_sensor_samples(self, primary_sample: SensorSample) -> dict[str, SensorSample]:
        samples: dict[str, SensorSample] = {primary_sample.name: primary_sample}
        for name in self._sensor_manager.names:
            if name == primary_sample.name:
                continue
            sensor = self._sensor_manager.get(name)
            try:
                samples[name] = sensor.read()
            except Exception as exc:  # pragma: no cover - defensive logging
                logging.warning("Failed to read data from sensor '%s': %s", name, exc)
        return samples

    def run(self) -> Iterator[ActuatorCommand]:
        logging.info("Starting autonomous pilot loop")
        self._running = True
        self._start_time = time.time()
        self._sensor_manager.start_all()
        primary_stream = self._sensor_manager.stream()
        for sample in primary_stream:
            if not self._running:
                break
            if not sample.ok or sample.data is None:
                logging.warning("Failed to read data from sensor '%s'", sample.name)
                continue
            frame = sample.data

            sensor_samples = self._collect_sensor_samples(sample)
            localization_state = self._localization.update(sensor_samples)
            self._latest_localization = localization_state
            lidar_samples = [
                sensor_samples[name]
                for name in self._lidar_names
                if name in sensor_samples and sensor_samples[name].ok
            ]

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

            guardian_decision = self._guardian.evaluate(
                perception_summary,
                decision,
                self._mode_manager.current_mode,
                lidar_samples=lidar_samples,
            )
            self._latest_guardian = guardian_decision

            if guardian_decision.force_stop:
                decision = replace(
                    decision,
                    desired_speed=0.0,
                    hazard_level=max(decision.hazard_level, 1.0),
                    enforced_stop=True,
                )
            elif guardian_decision.speed_scale < 0.999:
                decision = replace(
                    decision,
                    desired_speed=decision.desired_speed * guardian_decision.speed_scale,
                )

            mode_context = self._mode_manager.apply(decision, guardian_decision)
            self._latest_mode_context = mode_context
            decision = replace(
                decision,
                desired_speed=mode_context.desired_speed,
                enforced_stop=decision.enforced_stop or mode_context.force_stop,
                mode=mode_context.mode,
            )

            decision = self._summon_planner.adjust(
                decision,
                mode_context.mode,
                localization_state,
            )

            command = self._controller.command(decision)

            self._latest_decision = decision
            self._latest_directive = advisor_directive

            if self.config.visualize or self.config.video_output_path:
                self._visualize(frame, perception_summary, decision, command)

            self._export_state(current_command, advisor_directive, decision)
            self._export_command(command, decision, guardian_decision)

            yield command

        self._sensor_manager.stop_all()
        self._release_video_writer()
        self._close_command_log()
        logging.info("Pilot loop terminated")

    def stop(self) -> None:
        self._running = False
        self._sensor_manager.stop_all()
        self._release_video_writer()
        self._close_command_log()

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
                "mode": decision.mode.value,
                "hazard_level": decision.hazard_level,
            }
            if self._latest_guardian:
                payload["guardian_alerts"] = list(self._latest_guardian.alerts)
            if self._latest_localization:
                payload["localization_quality"] = self._latest_localization.quality
                payload["localization_min_lidar"] = self._latest_localization.metadata.get("lidar_min_distance")
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
        self._ensure_video_writer(frame)

        visual = frame.copy()
        visual = self._detector.draw_detections(visual, perception.objects)

        text_lines = [
            f"steer={command.steer:+.2f}",
            f"throttle={command.throttle:.2f}",
            f"brake={command.brake:.2f}",
            f"mode={decision.mode.value}",
            f"hazard={decision.hazard_level:.2f}",
        ]
        if decision.directive:
            text_lines.append(f"advisor: {decision.directive}")
        text_lines.append(f"goal: {decision.goal_context}")
        if self._latest_guardian and self._latest_guardian.alerts:
            text_lines.append(f"guardian: {','.join(self._latest_guardian.alerts)}")
        for idx, text in enumerate(text_lines):
            cv2.putText(visual, text, (10, 30 + idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        self._draw_actuator_bars(visual, command)
        self._draw_steering_vector(visual, command)

        if self._video_writer:
            self._video_writer.write(visual)

        if self.config.visualize:
            cv2.imshow("AutonomyPilot", visual)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.stop()

    def _draw_actuator_bars(self, frame: np.ndarray, command: ActuatorCommand) -> None:
        bar_width = 200
        bar_height = 12
        padding = 10
        origin_y = frame.shape[0] - 2 * (bar_height + padding) - 20

        def draw_bar(origin, value, color, label):
            x, y = origin
            cv2.rectangle(frame, (x, y), (x + bar_width, y + bar_height), (80, 80, 80), 1)
            fill_width = int(bar_width * max(0.0, min(1.0, value)))
            cv2.rectangle(frame, (x, y), (x + fill_width, y + bar_height), color, -1)
            cv2.putText(frame, f"{label}: {value:.2f}", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        draw_bar((10, origin_y), command.throttle, (0, 255, 255), "throttle")
        draw_bar((10, origin_y + bar_height + padding), command.brake, (0, 0, 255), "brake")

    def _draw_steering_vector(self, frame: np.ndarray, command: ActuatorCommand) -> None:
        height, width = frame.shape[:2]
        base_point = (width // 2, height - 60)
        max_length = min(width, height) // 3
        angle = command.steer * (math.pi / 3)
        end_x = int(base_point[0] + max_length * math.sin(angle))
        end_y = int(base_point[1] - max_length * math.cos(angle))
        cv2.arrowedLine(frame, base_point, (end_x, end_y), (255, 255, 0), 4, tipLength=0.1)
        cv2.putText(
            frame,
            "steering",
            (base_point[0] - 50, base_point[1] + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )

    def _ensure_video_writer(self, frame: np.ndarray) -> None:
        if self._video_writer or not self.config.video_output_path:
            return
        height, width = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(
            str(self.config.video_output_path),
            fourcc,
            float(self.config.camera_fps),
            (width, height),
        )
        if not video_writer.isOpened():
            raise RuntimeError(f"Unable to open video writer at {self.config.video_output_path}")
        self._video_writer = video_writer

    def _release_video_writer(self) -> None:
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None

    def _ensure_command_log(self) -> None:
        if self._command_log_writer or not self.config.command_log_path:
            return
        file_handle = self.config.command_log_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(file_handle)
        writer.writerow(
            [
                "time_s",
                "mode",
                "desired_speed_mps",
                "hazard_level",
                "steer",
                "throttle",
                "brake",
                "steer_hw",
                "throttle_hw",
                "brake_hw",
                "guardian_alerts",
                "localization_quality",
                "localization_min_lidar",
            ]
        )
        self._command_log_file = file_handle
        self._command_log_writer = writer

    def _export_command(
        self,
        command: ActuatorCommand,
        decision: NavigationDecision,
        guardian_decision: GuardianDecision,
    ) -> None:
        if not self.config.command_log_path:
            return
        self._ensure_command_log()
        if not self._command_log_writer or self._start_time is None:
            return
        elapsed = time.time() - self._start_time
        steer_hw, throttle_hw, brake_hw = self.map_to_hardware(command)
        self._command_log_writer.writerow(
            [
                f"{elapsed:.3f}",
                decision.mode.value,
                f"{decision.desired_speed:.3f}",
                f"{decision.hazard_level:.3f}",
                f"{command.steer:.4f}",
                f"{command.throttle:.4f}",
                f"{command.brake:.4f}",
                f"{steer_hw:.2f}",
                f"{throttle_hw:.2f}",
                f"{brake_hw:.2f}",
                ",".join(guardian_decision.alerts),
                f"{self._latest_localization.quality:.3f}" if self._latest_localization else "",
                f"{self._latest_localization.metadata.get('lidar_min_distance', '')}" if self._latest_localization else "",
            ]
        )
        if self._command_log_file:
            self._command_log_file.flush()

    def _close_command_log(self) -> None:
        if self._command_log_file is not None:
            self._command_log_file.close()
            self._command_log_file = None
            self._command_log_writer = None

    def map_to_hardware(self, command: ActuatorCommand) -> tuple[float, float, float]:
        """Convert normalized commands to MCU-facing units using the configured mapping."""
        mapping = self.config.hardware_mapping
        steer_hw = mapping.steer_center + mapping.steer_range * command.steer
        throttle_hw = mapping.throttle_offset + mapping.throttle_scale * command.throttle
        brake_hw = mapping.brake_offset + mapping.brake_scale * command.brake
        return steer_hw, throttle_hw, brake_hw


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
            mode_value = decision.mode.value if decision else pilot.mode_manager.current_mode.value
            guardian_alerts = ",".join(pilot.latest_guardian.alerts) if pilot.latest_guardian else ""
            localization_state = pilot.latest_localization
            localization_quality = f"{localization_state.quality:.2f}" if localization_state else ""
            steer_hw, throttle_hw, brake_hw = pilot.map_to_hardware(command)
            print(
                " ".join(
                    [
                        f"time={elapsed:.2f}s",
                        f"steer={command.steer:+.3f}",
                        f"throttle={command.throttle:.3f}",
                        f"brake={command.brake:.3f}",
                        f"steer_hw={steer_hw:.1f}",
                        f"throttle_hw={throttle_hw:.1f}",
                        f"brake_hw={brake_hw:.1f}",
                        f"mode={mode_value}",
                        f"directive=\"{directive}\"",
                        f"goal=\"{goal_context}\"",
                        f"guardian=\"{guardian_alerts}\"",
                        f"loc_quality={localization_quality}",
                    ]
                ),
                flush=True,
            )

    logging.info("Autonomy run complete")


def parse_args(argv: Optional[list[str]] = None) -> PilotConfig:
    import argparse

    parser = argparse.ArgumentParser(description="Autonomous scooter pilot")
    parser.add_argument("--camera", default=0, help="Camera source index or path")
    parser.add_argument(
        "--camera-probe",
        action="append",
        default=[],
        metavar="SOURCE",
        help="Additional camera sources to probe (repeatable)",
    )
    parser.add_argument(
        "--camera-csi",
        type=int,
        default=None,
        help="Use Jetson CSI camera with the given sensor ID (overrides --camera)",
    )
    parser.add_argument(
        "--camera-gstreamer",
        action="store_true",
        help="Prefer the GStreamer backend when probing cameras",
    )
    parser.add_argument(
        "--camera-max-probe",
        type=int,
        default=6,
        help="Number of incremental indices to try when --camera auto is used",
    )
    parser.add_argument(
        "--camera-flip",
        type=int,
        default=0,
        help="nvvidconv flip-method value applied to Jetson CSI pipelines",
    )
    parser.add_argument("--sensor", default="camera", help="Primary sensor adapter key (default: camera)")
    parser.add_argument(
        "--sensor-arg",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Additional sensor-specific arguments (repeatable)",
    )
    parser.add_argument(
        "--sensors-config",
        default=None,
        help="Optional JSON file that lists sensor specifications (overrides --sensor/--sensor-arg)",
    )
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
    parser.add_argument(
        "--video-output",
        default=None,
        help="Path to write an annotated video with detections and actuator overlays",
    )
    parser.add_argument("--command-log", default=None, help="CSV file to record actuator commands for MCU integration")
    parser.add_argument(
        "--mode",
        default=VehicleMode.CRUISE.value,
        choices=[VehicleMode.IDLE.value, VehicleMode.CRUISE.value, VehicleMode.SUMMON.value],
        help="Initial operating mode",
    )
    parser.add_argument(
        "--cruise-speed",
        type=float,
        default=4.5,
        help="Max speed (m/s) when operating in cruise mode",
    )
    parser.add_argument(
        "--summon-speed",
        type=float,
        default=3.0,
        help="Max speed (m/s) when operating in summon mode",
    )
    parser.add_argument(
        "--guardian-slowdown",
        type=float,
        default=0.5,
        help="Hazard threshold for proactive slowdown (0-1)",
    )
    parser.add_argument(
        "--guardian-emergency",
        type=float,
        default=0.85,
        help="Hazard threshold for emergency stop (0-1)",
    )
    parser.add_argument(
        "--guardian-min-speed-scale",
        type=float,
        default=0.2,
        help="Minimum speed scale applied during guardian slowdowns",
    )
    parser.add_argument(
        "--guardian-lidar-slowdown",
        type=float,
        default=2.5,
        help="LiDAR distance (m) that triggers proactive slowdowns",
    )
    parser.add_argument(
        "--guardian-lidar-stop",
        type=float,
        default=1.0,
        help="LiDAR distance (m) that forces an emergency stop",
    )
    parser.add_argument(
        "--steer-center",
        type=float,
        default=1500.0,
        help="Center value for steering output (e.g., servo PWM microseconds)",
    )
    parser.add_argument(
        "--steer-range",
        type=float,
        default=400.0,
        help="Range applied to steering command (center +/- range maps steer=-1/+1)",
    )
    parser.add_argument(
        "--throttle-scale",
        type=float,
        default=255.0,
        help="Scale factor applied to normalized throttle before sending to the MCU",
    )
    parser.add_argument(
        "--throttle-offset",
        type=float,
        default=0.0,
        help="Offset added to throttle output after scaling",
    )
    parser.add_argument(
        "--brake-scale",
        type=float,
        default=255.0,
        help="Scale factor applied to normalized brake before sending to the MCU",
    )
    parser.add_argument(
        "--brake-offset",
        type=float,
        default=0.0,
        help="Offset added to brake output after scaling",
    )

    args = parser.parse_args(argv)
    camera_source = f"csi://{args.camera_csi}" if args.camera_csi is not None else args.camera

    def _coerce_arg(value: str):
        lower = value.lower()
        if lower in ("true", "false"):
            return lower == "true"
        for cast in (int, float):
            try:
                return cast(value)
            except ValueError:
                continue
        return value

    camera_probe_sources = [_coerce_arg(value) for value in args.camera_probe]

    sensor_specs: list[SensorSpec] = []
    sensor_args: dict = {}
    if args.sensors_config:
        sensors_path = Path(args.sensors_config).expanduser()
        try:
            payload = json.loads(sensors_path.read_text())
        except OSError as exc:
            parser.error(f"Unable to read sensors config: {exc}")
        except json.JSONDecodeError as exc:
            parser.error(f"Invalid JSON in sensors config: {exc}")
        if not isinstance(payload, list):
            parser.error("Sensors config must be a JSON list of objects")
        for entry in payload:
            if not isinstance(entry, dict) or "adapter" not in entry:
                parser.error("Each sensor definition must include an 'adapter' field")
            spec_args = entry.get("args", {})
            if not isinstance(spec_args, dict):
                parser.error("Sensor 'args' field must be an object mapping")
            sensor_specs.append(
                SensorSpec(
                    adapter=str(entry["adapter"]),
                    name=entry.get("name"),
                    args=spec_args,
                )
            )
    else:
        for item in args.sensor_arg:
            if "=" not in item:
                parser.error(f"Invalid --sensor-arg '{item}', expected KEY=VALUE")
            key, raw_value = item.split("=", 1)
            sensor_args[key] = _coerce_arg(raw_value)
        if args.sensor == "camera":
            if camera_probe_sources and "probe_sources" not in sensor_args:
                sensor_args["probe_sources"] = camera_probe_sources
            if args.camera_gstreamer and "prefer_gstreamer" not in sensor_args:
                sensor_args["prefer_gstreamer"] = True
            if "gstreamer_flip_method" not in sensor_args:
                sensor_args["gstreamer_flip_method"] = args.camera_flip
            if "max_probe_sources" not in sensor_args:
                sensor_args["max_probe_sources"] = args.camera_max_probe

    command_file = Path(args.command_file).expanduser() if args.command_file else None
    advisor_state_path = Path(args.advisor_state).expanduser() if args.advisor_state else None
    command_state_path = Path(args.command_state).expanduser() if args.command_state else None
    video_output_path = Path(args.video_output).expanduser() if args.video_output else None
    command_log_path = Path(args.command_log).expanduser() if args.command_log else None
    hardware_mapping = HardwareMapping(
        steer_center=args.steer_center,
        steer_range=args.steer_range,
        throttle_offset=args.throttle_offset,
        throttle_scale=args.throttle_scale,
        brake_offset=args.brake_offset,
        brake_scale=args.brake_scale,
    )
    mode_manager_config = ModeManagerConfig(
        cruise_speed_limit=args.cruise_speed,
        summon_speed_limit=args.summon_speed,
        idle_speed_limit=0.0,
    )
    guardian_config = GuardianConfig(
        slowdown_hazard_threshold=args.guardian_slowdown,
        emergency_hazard_threshold=args.guardian_emergency,
        minimum_speed_scale=args.guardian_min_speed_scale,
        lidar_slowdown_distance=args.guardian_lidar_slowdown,
        lidar_stop_distance=args.guardian_lidar_stop,
    )
    initial_mode = VehicleMode(args.mode)

    return PilotConfig(
        camera_source=camera_source,
        camera_width=args.width,
        camera_height=args.height,
        camera_fps=args.fps,
        camera_probe_sources=camera_probe_sources,
        camera_prefer_gstreamer=True if args.camera_gstreamer else None,
        camera_gstreamer_flip=args.camera_flip,
        camera_max_probe_sources=args.camera_max_probe,
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
        video_output_path=video_output_path,
        command_log_path=command_log_path,
        hardware_mapping=hardware_mapping,
        sensor_type=args.sensor,
        sensor_args=sensor_args,
        sensors=sensor_specs,
        mode_manager_config=mode_manager_config,
        guardian_config=guardian_config,
        initial_mode=initial_mode,
    )


def main(argv: Optional[list[str]] = None) -> None:
    config = parse_args(argv)
    run_pilot(config)


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    main(sys.argv[1:])
