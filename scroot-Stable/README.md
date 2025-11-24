# Minimal YOLO Pilot (Python 3.8+)

A tiny, CPU-friendly pilot that reads a USB/CSI camera, runs YOLO object detection, converts detections into steering/throttle/brake, and optionally publishes the actuator triplet to ROS. Everything lives in `autonomy/` with no extra tooling.

Run from this folder after installing dependencies:

```bash
python -m autonomy.pilot --model yolov8n.pt --device auto --camera 0 --display --log actuator_log.jsonl
```

See `REPORT.md` for full requirements, setup, runtime behavior, and troubleshooting.
