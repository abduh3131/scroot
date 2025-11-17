# Development Report

## Objective
Keep the scooter pilot fast on Python 3.8+ laptops and Jetson deployments while making it lane-aware, CPU-friendly, and easier to test offline.

## Key Enhancements

1. **Lane-Aware Navigation**
   - `autonomy/perception/lane_detection.py` now feeds curvature/offset data straight into `Navigator.plan`, so the steering bias actively re-centers inside any detected lane or shoulder instead of just annotating overlays.
   - The navigator metadata exposes the applied lane bias/weight, and arbiter reason tags (`lane_bias_right`, etc.) continue to track corrections while lane-induced hazards are capped at 0.18 so the pilot backs off smoothly instead of panic-braking.

2. **CPU/Advisor Toggles**
   - `ObjectDetectorConfig` gained a `device` field, the CLI exposes `--device {auto,cpu,cuda,quadro_p520}`, and the GUI’s Acceleration Mode dropdown lets you force YOLO to stay on CPU, insist on CUDA, or engage a Quadro P520 compatibility mode that locks YOLO to `cuda:0` for legacy laptops.
   - An **Enable Advisor** checkbox controls the riding companion; when disabled the pilot skips narration entirely.

3. **Media Test Output Control**
   - The Media Test tab accepts a custom folder or filename, ensures the directory exists, and records overlays wherever you choose (falling back to `logs/media_tests/` when left blank).
   - The log now states the exact save path, the worker writes `.mp4` overlays even during CPU-only runs, and a **Stop** button cancels the export, removes any partial file, and re-enables the start button so you can switch clips immediately.

4. **Lane Calibration & Faint-Line Handling**
   - The Launch tab now includes a **Lane Alignment & Warp** box with an auto horizon trim toggle, manual tilt field, and four normalized trapezoid points so any scooter camera can keep the translucent corridor glued to the road.
   - The lane detector fuses CLAHE, HSV/LAB masks, Canny, adaptive thresholds, and a “search around previous poly” fallback so faint or oddly colored markings still influence the navigator.

5. **Documentation Refresh**
   - README, GUI guide, and both reports describe the acceleration toggle, advisor switch, lane calibration workflow, and lane-aware steering logic in plain English so operators can trace exactly how and why the pilot follows lane paint.

## Testing Notes

- Runtime validation within this environment is limited because GPU-accelerated models (YOLO/Torch) are not available. The code has been structured to fail fast if required packages are missing, and the launcher explicitly checks for them before execution.

## Next Steps

- Integrate localization and mapping data to fulfill “drive to this location” requests with metric precision.
- Collect annotated lane datasets to tune the perspective warp per vehicle class.
- Add telemetry unit tests or simulation harnesses once Torch/Tk can be stubbed or mocked.
