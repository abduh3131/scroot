from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from .actuator_output import ActuatorPublisher
from .camera import CameraConfig, FrameSource
from .detector import YOLODetector
from .navigator import Navigator, NavigatorConfig


def overlay(frame: np.ndarray, steering: float, throttle: float, brake: float) -> np.ndarray:
    annotated = frame.copy()
    status = f"steer={steering:+.2f} throttle={throttle:.2f} brake={brake:.2f}"
    cv2.putText(
        annotated,
        status,
        (10, annotated.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return annotated


def run(
    model_path: str,
    camera_config: CameraConfig,
    navigator_config: NavigatorConfig,
    log_path: Optional[Path],
    ros_topic: Optional[str],
    device: str,
    display: bool,
) -> None:
    detector = YOLODetector(model_path=model_path, device=device)
    navigator = Navigator(navigator_config)
    publisher = ActuatorPublisher(log_path=log_path, ros_topic=ros_topic)

    with FrameSource(camera_config).frames() as frames:
        for frame in frames:
            detections = detector.infer(frame)
            command = navigator.plan(detections, frame_shape=frame.shape[:2])
            publisher.publish(command)

            if display:
                annotated = overlay(frame, command.steering, command.throttle, command.brake)
                for det in detections:
                    x1, y1, x2, y2 = map(int, det.bbox)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 160, 255), 2)
                    cv2.putText(
                        annotated,
                        f"{det.label}:{det.confidence:.2f}",
                        (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )
                cv2.imshow("pilot", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple YOLO-based pilot for ROS/Jetson and desktop")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO model path or name (default: yolov8n.pt)")
    parser.add_argument("--device", default="auto", help="Inference device: auto/cpu/cuda (default: auto)")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--width", type=int, default=640, help="Capture width")
    parser.add_argument("--height", type=int, default=480, help="Capture height")
    parser.add_argument("--fps", type=int, default=30, help="Capture FPS")
    parser.add_argument("--pipeline", help="Optional GStreamer pipeline string for Jetson/CSI cams")
    parser.add_argument("--log", type=Path, help="JSONL log file for actuator commands")
    parser.add_argument("--ros-topic", help="Publish [steer, throttle, brake] to this ROS topic if rospy is available")
    parser.add_argument("--display", action="store_true", help="Show an annotated preview window")
    parser.add_argument("--cruise-throttle", type=float, default=0.35, help="Baseline throttle command")
    parser.add_argument("--brake-area", type=float, default=0.22, help="Frame fraction that triggers braking")
    parser.add_argument("--steer-gain", type=float, default=0.8, help="Steering gain for obstacle avoidance")
    args = parser.parse_args()

    camera_cfg = CameraConfig(
        device=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        pipeline=args.pipeline,
    )
    navigator_cfg = NavigatorConfig(
        cruise_throttle=args.cruise_throttle,
        brake_area_threshold=args.brake_area,
        steer_gain=args.steer_gain,
    )
    run(
        model_path=args.model,
        camera_config=camera_cfg,
        navigator_config=navigator_cfg,
        log_path=args.log,
        ros_topic=args.ros_topic,
        device=args.device,
        display=args.display,
    )


if __name__ == "__main__":
    main()
