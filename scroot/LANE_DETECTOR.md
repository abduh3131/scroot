# Lane Detector Architecture and Rationale

## Overview
The lane detector that powers the scooter advisor lives in
`autonomy/perception/lane_detection.py`. It is intentionally lightweight so it
runs well on CPU-only laptops and embedded systems while still giving the
advisor enough context to bias toward safer positions in the lane. The
implementation favors deterministic OpenCV primitives that are available on any
platform we support.

## Processing Pipeline
1. **Pre-processing** – Each camera frame is converted to grayscale and blurred
   with a 5×5 Gaussian kernel to suppress sensor noise and compression artifacts.
2. **Edge Extraction** – We apply a configurable Canny filter (`canny_low=60`,
   `canny_high=150`) to highlight lane markings and curbs.
3. **Region of Interest Mask** – Only the lower portion of the image is
   considered. The mask starts at 40 % from the bottom of the frame so sky and
   distant scenery do not trigger the detector.
4. **Hough Line Detection** – A probabilistic Hough transform searches for line
   segments in the masked edge map using the configured `hough_threshold`,
   `min_line_length`, and `max_line_gap` values.
5. **Left/Right Aggregation** – Detected line segments are separated by slope.
   Negative slopes vote toward the left boundary and positive slopes vote toward
   the right boundary. Shallow slopes (|m| < 0.3) are discarded to avoid noise.
6. **Averaging and Projection** – We compute the average slope/intercept for
   left and right collections and project them back into the frame to draw the
   lane borders and to compute a vanishing point.
7. **Offset and Confidence** – The detector reports the scooter’s lateral offset
   from the lane centre in pixels and as a normalized ratio. Confidence grows
   with the number of supporting segments and is smoothed over time.
8. **Temporal Smoothing** – The previous observation is blended with the current
   detection (`smoothing=0.25`) to reduce jitter and provide continuity when a
   single frame fails to find both lane sides.

## Why This Approach Works
- **CPU Friendly** – The entire pipeline uses OpenCV operations that execute in a
  few milliseconds on mid-range CPUs. There are no neural network dependencies,
  so low-end hosts run comfortably.
- **Deterministic Behaviour** – Every parameter is explicit and bounded, which
  keeps the advisor’s arbitration predictable and debuggable. This is important
  for safety certification.
- **Graceful Degradation** – If either side of the lane disappears the detector
  returns the last confident estimate with a decaying confidence score. The
  pilot treats low-confidence readings as a hint rather than a mandate, so the
  scooter continues safely.
- **Vehicle Envelope Awareness** – The lane offset feeds the vehicle envelope and
  advisor logic so the system knows when lateral clearance is tight. Combined
  with the vehicle width entered during setup, this stops the scooter from
  nudging into obstacles or curbs.

## Extensibility Hooks
- Configuration values live in `LaneDetectorConfig` making it easy to tweak
  thresholds per deployment profile.
- The detector exposes raw points for both lane edges and a vanishing point.
  These can be rendered in the GUI overlay or consumed by future planners.
- Because the implementation uses standard OpenCV primitives, it can be swapped
  out for GPU-accelerated alternatives on higher-end devices without changing
  the downstream APIs.

## Testing Notes
During development the detector is exercised on recorded rides and synthetic
scenes that stress wide, narrow, and partially occluded lanes. The smoothing and
confidence heuristics were tuned to prefer stability over reacting to every
single-frame anomaly, which keeps the safety arbiter from issuing unnecessary
fail-stops.
