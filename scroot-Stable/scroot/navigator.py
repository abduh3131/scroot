"""Minimal self-driving navigator using YOLO detections and actuator overlays."""
from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import serial
from ultralytics import YOLO

ActuatorValues = Tuple[float, float, float]


class ArduinoController:
    """Send actuator values to an attached Arduino over serial."""

    def __init__(self, port: str | None, baud_rate: int = 115200) -> None:
        self.port = port
        self.baud_rate = baud_rate
        self._serial = serial.Serial(port, baud_rate, timeout=0.1) if port else None

    def send(self, actuators: ActuatorValues) -> None:
        message = f"{actuators[0]:.3f},{actuators[1]:.3f},{actuators[2]:.3f}\n"
        if self._serial:
            self._serial.write(message.encode("utf-8"))
        logging.debug("Sent to Arduino: %s", message.strip())

    def close(self) -> None:
        if self._serial:
            self._serial.close()


class SimpleNavigator:
    """Steers and modulates throttle/brake to avoid detected obstacles."""

    def __init__(
        self,
        model_path: str,
        device: str,
        cruise_speed: float = 0.35,
        caution_area: float = 0.05,
        stop_area: float = 0.18,
        conf_threshold: float = 0.35,
    ) -> None:
        self.model = YOLO(model_path)
        self.model.to(device)
        self.cruise_speed = cruise_speed
        self.caution_area = caution_area
        self.stop_area = stop_area
        self.conf_threshold = conf_threshold

    def _normalize_offset(self, x_center: float, frame_width: int) -> float:
        half_width = frame_width / 2.0
        return (x_center - half_width) / half_width

    def compute_actuators(self, result, frame_shape: Tuple[int, int, int]) -> ActuatorValues:
        height, width, _ = frame_shape
        steering = 0.0
        throttle = self.cruise_speed
        brake = 0.0

        boxes = result.boxes if result is not None else None
        if boxes is None or len(boxes) == 0:
            return (steering, throttle, brake)

        for box in boxes:
            conf = float(box.conf[0])
            if conf < self.conf_threshold:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            area = ((x2 - x1) * (y2 - y1)) / float(width * height)
            offset = self._normalize_offset((x1 + x2) / 2.0, width)

            steering -= offset * min(1.0, area * 3.0)
            throttle = max(0.1, throttle * (1.0 - min(area / self.caution_area, 1.0)))

            if area >= self.stop_area:
                throttle = 0.0
                brake = 1.0
            elif area >= self.caution_area:
                brake = max(brake, min(1.0, (area - self.caution_area) / (self.stop_area - self.caution_area)))

        steering = float(np.clip(steering, -1.0, 1.0))
        throttle = float(np.clip(throttle, 0.0, 1.0))
        brake = float(np.clip(brake, 0.0, 1.0))
        return (steering, throttle, brake)


def draw_overlays(frame: np.ndarray, result, actuators: ActuatorValues, calibration: bool) -> np.ndarray:
    display = frame.copy()
    height, width, _ = display.shape

    if result is not None and result.boxes is not None:
        for box in result.boxes:
            conf = float(box.conf[0])
            if conf < 0.25:
                continue
            cls_id = int(box.cls[0]) if box.cls is not None else -1
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 200, 255), 2)
            label = f"{cls_id if cls_id >= 0 else 'obj'} {conf:.2f}"
            cv2.putText(display, label, (x1, max(y1 - 4, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

    steering, throttle, brake = actuators
    status_text = f"({steering:+.2f}, {throttle:.2f}, {brake:.2f})"
    if calibration:
        status_text += " | CALIBRATION"
    cv2.putText(display, status_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(display, status_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    bar_height = int(height * 0.25)
    bar_width = 24
    origin_y = height - 20
    throttle_top = origin_y - int(bar_height * throttle)
    brake_top = origin_y - int(bar_height * brake)
    cv2.rectangle(display, (20, origin_y - bar_height), (20 + bar_width, origin_y), (60, 60, 60), 1)
    cv2.rectangle(display, (20, throttle_top), (20 + bar_width, origin_y), (0, 255, 0), -1)
    cv2.putText(display, "TH", (20, origin_y - bar_height - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.rectangle(display, (60, origin_y - bar_height), (60 + bar_width, origin_y), (60, 60, 60), 1)
    cv2.rectangle(display, (60, brake_top), (60 + bar_width, origin_y), (0, 0, 255), -1)
    cv2.putText(display, "BR", (60, origin_y - bar_height - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

    center_y = int(height * 0.75)
    center_x = width // 2
    steering_offset = int(steering * (width // 3))
    cv2.line(display, (center_x - width // 4, center_y), (center_x + width // 4, center_y), (200, 200, 200), 1)
    cv2.circle(display, (center_x, center_y), 6, (255, 255, 255), 1)
    cv2.line(display, (center_x, center_y - 10), (center_x + steering_offset, center_y), (0, 180, 255), 3)

    return display


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Autonomous navigator with YOLO overlays and Arduino output.")
    parser.add_argument("--input", required=True, help="Path to an MP4 video file.")
    parser.add_argument("--model", default="yolov8n.pt", help="Path to a lightweight YOLO model file (default: yolov8n.pt).")
    parser.add_argument("--serial-port", default=None, help="Serial port for the Arduino (e.g., /dev/ttyACM0). If omitted, actuator values are only logged.")
    parser.add_argument("--baud-rate", type=int, default=115200, help="Baud rate for the Arduino serial connection.")
    parser.add_argument("--device", default="cpu", help="Computation device for YOLO (e.g., cpu, cuda:0).")
    parser.add_argument("--display", action="store_true", help="Show a GUI window with the live overlay. Off by default.")
    parser.add_argument("--output", default=None, help="Optional path to save the annotated video.")
    parser.add_argument("--calibration", type=float, default=3.0, help="Seconds to output zeroed actuators for calibration.")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(message)s")

    video_path = Path(args.input)
    if not video_path.exists():
        raise FileNotFoundError(f"Input video not found: {video_path}")

    controller = ArduinoController(args.serial_port, args.baud_rate)
    navigator = SimpleNavigator(args.model, device=args.device)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1280
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720

    writer = None
    if args.output:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(args.output), fourcc, fps, (width, height))

    start_time = time.time()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            elapsed = time.time() - start_time
            calibration_active = elapsed < args.calibration
            result = None
            if not calibration_active:
                result = navigator.model.predict(frame, imgsz=640, device=args.device, verbose=False)[0]
                actuators = navigator.compute_actuators(result, frame.shape)
            else:
                actuators = (0.0, 0.0, 0.0)

            controller.send(actuators)
            overlay_frame = draw_overlays(frame, result, actuators, calibration_active)
            logging.info(
                "(%.3f, %.3f, %.3f)%s",
                actuators[0],
                actuators[1],
                actuators[2],
                " (calibration)" if calibration_active else "",
            )

            if writer is not None:
                writer.write(overlay_frame)

            if args.display:
                cv2.imshow("Navigator", overlay_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        controller.close()
        cap.release()
        if writer is not None:
            writer.release()
        if args.display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
