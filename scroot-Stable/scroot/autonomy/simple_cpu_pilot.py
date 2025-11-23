from __future__ import annotations

import argparse
import logging
import signal
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2

from autonomy.control.actuator_output import ActuatorOutput
from autonomy.perception.object_detection import ObjectDetector, ObjectDetectorConfig
from autonomy.sensors.camera import CameraSensor
from autonomy.utils.data_structures import ActuatorCommand, PerceptionSummary


@dataclass
class SimplePilotConfig:
    camera_source: int | str = 0
    width: int = 640
    height: int = 360
    fps: int = 15
    model_name: str = "yolov8n.pt"
    confidence: float = 0.25
    iou: float = 0.4
    visualize: bool = False
    log_path: Path = Path("logs/simple_actuators.jsonl")
    serial_device: Optional[str] = None
    auto_serial: bool = True
    baud_rate: int = 115200
    steering_gain: float = 0.7
    brake_area_ratio: float = 0.18
    idle_throttle: float = 0.45
    smooth_factor: float = 0.2


class CommandSmoother:
    def __init__(self, factor: float) -> None:
        self.factor = max(0.0, min(1.0, factor))
        self._previous: Optional[ActuatorCommand] = None

    def apply(self, command: ActuatorCommand) -> ActuatorCommand:
        if self._previous is None:
            self._previous = command
            return command

        blend = self.factor
        smoothed = ActuatorCommand(
            steer=self._previous.steer * (1 - blend) + command.steer * blend,
            throttle=self._previous.throttle * (1 - blend) + command.throttle * blend,
            brake=self._previous.brake * (1 - blend) + command.brake * blend,
        )

        self._previous = smoothed
        return smoothed


class SimpleSelfDrivingPilot:
    """CPU-only pilot that pairs YOLO detections with a basic avoidance policy."""

    def __init__(self, config: SimplePilotConfig) -> None:
        self.config = config
        self._camera = CameraSensor(
            source=config.camera_source,
            width=config.width,
            height=config.height,
            fps=config.fps,
        )
        self._detector = ObjectDetector(
            ObjectDetectorConfig(
                model_name=config.model_name,
                confidence_threshold=config.confidence,
                iou_threshold=config.iou,
                device="cpu",
            )
        )
        self._smoother = CommandSmoother(config.smooth_factor)
        self._actuator_output = ActuatorOutput(
            log_path=config.log_path,
            serial_device=config.serial_device,
            baud_rate=config.baud_rate,
            auto_discover_serial=config.auto_serial,
        )
        self._running = False

    def _compute_command(self, perception: PerceptionSummary) -> ActuatorCommand:
        if not perception.objects:
            return ActuatorCommand(steer=0.0, throttle=self.config.idle_throttle, brake=0.0)

        width, height = perception.frame_size
        frame_area = float(width * height)
        largest = max(perception.objects, key=lambda obj: obj.area)
        center_x = (largest.bbox[0] + largest.bbox[2]) * 0.5
        offset = (center_x - (width * 0.5)) / float(width)
        steer = -offset * self.config.steering_gain

        proximity = min(1.0, largest.area / (frame_area * self.config.brake_area_ratio))
        throttle = max(0.0, self.config.idle_throttle * (1.0 - proximity))
        brake = min(1.0, proximity * 1.25)

        return ActuatorCommand(steer=steer, throttle=throttle, brake=brake)

    def _render_overlay(
        self, frame, perception: PerceptionSummary, command: ActuatorCommand
    ):
        overlay = frame.copy()
        overlay = self._detector.draw_detections(overlay, perception.objects)
        summary = f"steer={command.steer:+.2f} th={command.throttle:.2f} br={command.brake:.2f}"
        cv2.putText(
            overlay,
            summary,
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (50, 220, 50),
            2,
        )
        return overlay

    def run(self) -> None:
        self._running = True
        for success, frame in self._camera.frames():
            if not self._running:
                break
            if not success or frame is None:
                logging.warning("Camera dropped a frame; continuing")
                continue

            perception = self._detector.detect(frame)
            command = self._compute_command(perception)
            command = self._smoother.apply(command)
            self._actuator_output.publish(command)

            if self.config.visualize:
                overlay = self._render_overlay(frame, perception, command)
                cv2.imshow("Simple Pilot", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        self.close()

    def close(self) -> None:
        self._running = False
        self._camera.close()
        self._actuator_output.close()
        cv2.destroyAllWindows()


def parse_args() -> SimplePilotConfig:
    parser = argparse.ArgumentParser(description="Lightweight CPU-only self driving loop")
    parser.add_argument("--camera", type=str, default="0", help="Camera index or video path")
    parser.add_argument("--width", type=int, default=640, help="Capture width")
    parser.add_argument("--height", type=int, default=360, help="Capture height")
    parser.add_argument("--fps", type=int, default=15, help="Capture frames per second")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="YOLO model file")
    parser.add_argument("--confidence", type=float, default=0.25, help="Detection confidence threshold")
    parser.add_argument("--iou", type=float, default=0.4, help="Detection IoU threshold")
    parser.add_argument("--visualize", action="store_true", help="Display camera feed with overlays")
    parser.add_argument("--log-path", type=Path, default=Path("logs/simple_actuators.jsonl"), help="File to append actuator commands")
    parser.add_argument("--serial", type=str, default=None, help="Serial device for MCU (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument("--no-auto-serial", action="store_true", help="Disable auto-discovery of a serial device on Jetson/Linux")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--steering-gain", type=float, default=0.7, help="Steering gain away from detected obstacle center")
    parser.add_argument("--brake-area-ratio", type=float, default=0.18, help="Area ratio that triggers full braking")
    parser.add_argument("--idle-throttle", type=float, default=0.45, help="Base throttle when no obstacles are present")
    parser.add_argument("--smooth-factor", type=float, default=0.2, help="Exponential smoothing factor for actuator commands")

    args = parser.parse_args()

    try:
        camera_source: int | str = int(args.camera)
    except ValueError:
        camera_source = args.camera

    serial_device = None if args.serial == "auto" else args.serial

    return SimplePilotConfig(
        camera_source=camera_source,
        width=args.width,
        height=args.height,
        fps=args.fps,
        model_name=args.model,
        confidence=args.confidence,
        iou=args.iou,
        visualize=args.visualize,
        log_path=args.log_path,
        serial_device=serial_device,
        auto_serial=not args.no_auto_serial,
        baud_rate=args.baud,
        steering_gain=args.steering_gain,
        brake_area_ratio=args.brake_area_ratio,
        idle_throttle=args.idle_throttle,
        smooth_factor=args.smooth_factor,
    )


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(message)s")
    config = parse_args()
    pilot = SimpleSelfDrivingPilot(config)

    def handle_interrupt(signum, frame):
        logging.info("Stopping pilot loop")
        pilot.close()

    signal.signal(signal.SIGINT, handle_interrupt)
    pilot.run()


if __name__ == "__main__":
    main()
