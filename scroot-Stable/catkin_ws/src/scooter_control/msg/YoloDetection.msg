# Defines a single YOLO object detection

# Class label of the detected object
string class_label

# Confidence score of the detection
float32 confidence

# Bounding box coordinates (in pixel space)
# top-left corner
int32 x_min
int32 y_min
# bottom-right corner
int32 x_max
int32 y_max
float32 distance   # estimated distance to object in meters, -1 if unknown
