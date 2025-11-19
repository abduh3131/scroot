# An array of YOLO detections from the dual_yolo_node

std_msgs/Header header

# Array of individual detections
YoloDetection[] detections

# Source of the detection (e.g., "yolov8n", "yolov8s", "fused")
string source_model
