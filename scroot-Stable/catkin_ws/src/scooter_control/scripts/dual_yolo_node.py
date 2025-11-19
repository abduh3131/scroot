#!/usr/bin/env python3

# ---- NumPy compatibility fix ----
import numpy as np
np.bool = getattr(np, "bool", np.bool_)

# ---- ROS & CV imports ----
import rospy
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

# ---- CUDA + TensorRT ----
import pycuda.driver as cuda
import tensorrt as trt
import threading
import time

# ---- RViz Markers ----
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# ---- ROS Messages ----
from scooter_control.msg import YoloDetection, YoloDetectionArray

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

# ---- COCO CLASS LIST ----
CLASS_NAMES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake",
    "chair","couch","potted plant","bed","dining table","toilet","tv","laptop",
    "mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush"
]

# ============================================================
#                 Simple SORT-like Tracker
# ============================================================

def iou_xyxy(a, b):
    """a,b: [x1,y1,x2,y2]"""
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    w = max(0, x2 - x1)
    h = max(0, y2 - y1)
    inter = w * h
    if inter <= 0:
        return 0.0
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return float(inter) / float(area_a + area_b - inter + 1e-6)


class SimpleSortTracker(object):
    """
    Lightweight IoU-based tracker:
      * keeps tracks with id + bbox
      * matches new boxes by IoU
      * creates new tracks for unmatched boxes
      * drops tracks after max_age missed frames
    No Kalman filter – just association.
    """

    def __init__(self, iou_thresh=0.3, max_age=10):
        self.iou_thresh = iou_thresh
        self.max_age = max_age
        self.next_id = 1
        self.tracks = []  # each: {id, bbox, age, miss}

    def update(self, boxes):
        track_ids = [-1] * len(boxes)
        if len(boxes) == 0:
            # Age out tracks
            for t in self.tracks:
                t["age"] += 1
                t["miss"] += 1
            self.tracks = [t for t in self.tracks if t["miss"] <= self.max_age]
            return []

        # IoU matrix tracks x boxes
        iou_matrix = np.zeros((len(self.tracks), len(boxes)), dtype=np.float32)
        for ti, t in enumerate(self.tracks):
            for bi, b in enumerate(boxes):
                iou_matrix[ti, bi] = iou_xyxy(t["bbox"], b)

        used_tracks = set()
        used_boxes = set()

        # Greedy matching
        while iou_matrix.size > 0:
            ti, bi = divmod(np.argmax(iou_matrix), iou_matrix.shape[1])
            best = iou_matrix[ti, bi]
            if best < self.iou_thresh:
                break
            if ti in used_tracks or bi in used_boxes:
                iou_matrix[ti, bi] = -1
                continue
            t = self.tracks[ti]
            t["bbox"] = boxes[bi]
            t["age"] += 1
            t["miss"] = 0
            track_ids[bi] = t["id"]
            used_tracks.add(ti)
            used_boxes.add(bi)
            iou_matrix[ti, :] = -1
            iou_matrix[:, bi] = -1

        # New tracks for unmatched boxes
        for bi, b in enumerate(boxes):
            if bi in used_boxes:
                continue
            tid = self.next_id
            self.next_id += 1
            self.tracks.append({
                "id": tid,
                "bbox": b,
                "age": 1,
                "miss": 0,
            })
            track_ids[bi] = tid

        # Age out unmatched tracks
        for idx, t in enumerate(self.tracks):
            if idx not in used_tracks:
                t["age"] += 1
                t["miss"] += 1
        self.tracks = [t for t in self.tracks if t["miss"] <= self.max_age]

        return track_ids


# ============================================================
#                  TensorRT Wrapper (unchanged)
# ============================================================

class TrtYoloModel:
    """Owns the CUDA context + TensorRT engine, runs inference."""

    def __init__(self, engine_path):
        cuda.init()
        self.device = cuda.Device(0)
        self.context = self.device.make_context()
        rospy.loginfo("[TRT] CUDA context created.")

        self.runtime = trt.Runtime(TRT_LOGGER)
        with open(engine_path, "rb") as f:
            engine_bytes = f.read()
        self.engine = self.runtime.deserialize_cuda_engine(engine_bytes)
        self.exec_ctx = self.engine.create_execution_context()

        self.bindings = [None] * self.engine.num_bindings
        self.inputs = []
        self.outputs = []

        for i in range(self.engine.num_bindings):
            shape = self.engine.get_binding_shape(i)
            size = trt.volume(shape)
            dtype = trt.nptype(self.engine.get_binding_dtype(i))

            host = cuda.pagelocked_empty(size, dtype)
            dev = cuda.mem_alloc(host.nbytes)

            self.bindings[i] = int(dev)
            entry = {"host": host, "device": dev, "index": i}

            if self.engine.binding_is_input(i):
                self.inputs.append(entry)
            else:
                self.outputs.append(entry)

        rospy.loginfo("[TRT] Engine + buffers ready.")

    def letterbox(self, img, new_shape=(640, 640)):
        h, w = img.shape[:2]
        target_h, target_w = new_shape

        scale = min(target_w / w, target_h / h)
        new_w, new_h = int(w * scale), int(h * scale)

        resized = cv2.resize(img, (new_w, new_h), cv2.INTER_AREA)
        padded = np.full((target_h, target_w, 3), 114, np.uint8)

        pad_x = (target_w - new_w) // 2
        pad_y = (target_h - new_h) // 2
        padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized

        blob = padded.astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)
        return np.ascontiguousarray(blob), scale, (new_w, new_h), (pad_x, pad_y)

    def infer(self, frame):
        blob, scale, resized, pads = self.letterbox(frame)

        inp = self.inputs[0]
        np.copyto(inp["host"], blob.ravel())
        cuda.memcpy_htod(inp["device"], inp["host"])

        self.exec_ctx.execute_v2(self.bindings)

        out = self.outputs[0]
        cuda.memcpy_dtoh(out["host"], out["device"])

        preds = out["host"].reshape(1, 84, 8400).transpose(0, 2, 1)[0]
        return preds, scale, resized, pads

    def decode(self, preds, img_shape, scale, resized, pads, conf=0.25):
        H, W = img_shape[:2]
        new_w, new_h = resized
        pad_x, pad_y = pads

        boxes = []
        scores = []
        classes = []
        for det in preds:
            cx, cy, w, h = det[:4]
            cls_scores = det[4:]
            sc = float(np.max(cls_scores))
            if sc < conf:
                continue
            cid = int(np.argmax(cls_scores))

            x1 = (cx - w / 2 - pad_x) / scale
            y1 = (cy - h / 2 - pad_y) / scale
            x2 = (cx + w / 2 - pad_x) / scale
            y2 = (cy + h / 2 - pad_y) / scale

            x1 = max(0, min(int(x1), W))
            y1 = max(0, min(int(y1), H))
            x2 = max(0, min(int(x2), W))
            y2 = max(0, min(int(y2), H))

            boxes.append([x1, y1, x2, y2])
            scores.append(sc)
            classes.append(cid)

        if len(boxes) == 0:
            return [], [], []
        boxes_xywh = [[b[0], b[1], b[2] - b[0], b[3] - b[1]] for b in boxes]
        idxs = cv2.dnn.NMSBoxes(boxes_xywh, scores, 0.0, 0.45)
        if len(idxs) == 0:
            return [], [], []

        idxs = idxs.flatten()
        return [boxes[i] for i in idxs], [scores[i] for i in idxs], [classes[i] for i in idxs]


# ============================================================
#           Dual YOLO Node (High-FPS + Tracking + Depth)
# ============================================================

class DualYoloNode:
    def __init__(self):
        rospy.init_node("dual_yolo_node")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_scan = None
        self.lock = threading.Lock()

        self.tracker = SimpleSortTracker(iou_thresh=0.3, max_age=10)

        self.pub_img = rospy.Publisher("/AI/annotated_image", Image, queue_size=1)
        self.pub_det = rospy.Publisher("/AI/detections", YoloDetectionArray, queue_size=1)
        self.pub_markers = rospy.Publisher("/AI/markers", MarkerArray, queue_size=1)

        self.model = TrtYoloModel("/home/jetson/yolov8n_fp16.engine")
        self.infer_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.infer_thread.start()

        # fused camera image from sensor_interface_node
        rospy.Subscriber("/sensor_hub/fused_image", Image, self.cam_cb, queue_size=1)
        # raw LiDAR scan from rplidarNode
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)

        rospy.loginfo("[DualYolo] High-FPS node running on /sensor_hub/fused_image with /scan fusion.")

    # ---------------- CAMERA CALLBACK ----------------
    def cam_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.latest_frame = (img, msg.header)

    # ---------------- SCAN CALLBACK ----------------
    def scan_cb(self, msg):
        with self.lock:
            self.latest_scan = msg

    # ---------------- DEPTH ESTIMATION ----------------
    def estimate_distance_for_box(self, box, scan, img_width):
        """
        Very simple placeholder fusion:
        - uses only the central beam of the LaserScan
        - returns distance in meters, or -1 if not available

        This is intentionally simple and safe; later we can map
        pixel x to scan angle for better alignment.
        """
        if scan is None:
            return -1.0
        if not scan.ranges:
            return -1.0
        try:
            ranges = np.array(scan.ranges, dtype=np.float32)
            valid = ranges[np.isfinite(ranges)]
            if valid.size == 0:
                return -1.0
            # use median of valid ranges around center index
            center_idx = len(ranges) // 2
            window = 5
            start = max(0, center_idx - window)
            end = min(len(ranges), center_idx + window + 1)
            window_vals = ranges[start:end]
            window_vals = window_vals[np.isfinite(window_vals)]
            if window_vals.size == 0:
                return float(np.median(valid))
            return float(np.median(window_vals))
        except Exception:
            return -1.0

    # ---------------- MARKER PUBLISHING ----------------
    def publish_markers(self, header, boxes, classes, distances, track_ids):
        marker_array = MarkerArray()
        now = rospy.Time.now()
        frame_id = header.frame_id if header.frame_id else "webcam_link"

        for i, b in enumerate(boxes):
            x1, y1, x2, y2 = b
            cls_id = classes[i]
            dist = distances[i] if i < len(distances) else -1.0
            tid = track_ids[i] if i < len(track_ids) else -1
            label = CLASS_NAMES[cls_id]

            # very rough 3D placement: straight ahead at distance 'dist'
            z = dist if dist > 0 else 1.0
            x = 0.0
            y = 0.0

            cube = Marker()
            cube.header.stamp = now
            cube.header.frame_id = frame_id
            cube.ns = "ai_obstacles"
            cube.id = int(tid) if tid >= 0 else i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position = Point(x=x, y=y, z=z)
            cube.pose.orientation.x = 0.0
            cube.pose.orientation.y = 0.0
            cube.pose.orientation.z = 0.0
            cube.pose.orientation.w = 1.0
            cube.scale.x = 0.3
            cube.scale.y = 0.3
            cube.scale.z = 0.3
            color_factor = (cls_id % 10) / 10.0
            cube.color.r = 1.0 - color_factor
            cube.color.g = color_factor
            cube.color.b = 0.3
            cube.color.a = 0.8
            cube.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(cube)

            text = Marker()
            text.header.stamp = now
            text.header.frame_id = frame_id
            text.ns = "ai_obstacle_labels"
            text.id = int(10000 + cube.id)
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=x, y=y, z=z + 0.5)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.3
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            dist_str = f"{dist:.2f}m" if dist > 0 else "?"
            text.text = f"ID {tid} {label} {dist_str}"
            text.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(text)

        self.pub_markers.publish(marker_array)

    # ---------------- INFERENCE THREAD ----------------
    def inference_loop(self):
        self.model.context.push()
        rospy.loginfo("[DualYolo] CUDA context attached to inference thread.")

        while not rospy.is_shutdown():
            with self.lock:
                data = self.latest_frame
                scan = self.latest_scan
                self.latest_frame = None

            if data is None:
                time.sleep(0.002)
                continue

            frame, header = data

            preds, scale, resized, pads = self.model.infer(frame)
            boxes, scores, classes = self.model.decode(
                preds, frame.shape, scale, resized, pads
            )

            # A) tracking
            track_ids = self.tracker.update(boxes) if len(boxes) > 0 else []

            # B/C) estimate distances per box using /scan
            distances = []
            img_height, img_width = frame.shape[:2]
            for b in boxes:
                d = self.estimate_distance_for_box(b, scan, img_width)
                distances.append(d)

            # Build YoloDetectionArray (with distance field)
            det_arr = YoloDetectionArray()
            det_arr.header = header
            det_arr.source_model = "yolov8n"

            for i, (b, s, c) in enumerate(zip(boxes, scores, classes)):
                det = YoloDetection()
                det.class_label = CLASS_NAMES[c]
                det.confidence = float(s)
                det.x_min, det.y_min, det.x_max, det.y_max = map(int, b)
                det.distance = float(distances[i]) if i < len(distances) else -1.0
                det_arr.detections.append(det)

            self.pub_det.publish(det_arr)

            # D) RViz markers
            self.publish_markers(header, boxes, classes, distances, track_ids)

            # Annotated image
            annotated = frame.copy()
            for i, (b, s, c) in enumerate(zip(boxes, scores, classes)):
                x1, y1, x2, y2 = b
                tid = track_ids[i] if i < len(track_ids) else -1
                dist = distances[i] if i < len(distances) else -1.0
                label = f"{tid}:{CLASS_NAMES[c]} {s:.2f}"
                if dist > 0:
                    label += f" {dist:.1f}m"
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    label,
                    (x1, max(0, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 0),
                    2,
                )

            msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            msg.header = header
            self.pub_img.publish(msg)

        self.model.context.pop()


# ---------------- MAIN ----------------
if __name__ == "__main__":
    node = DualYoloNode()
    rospy.spin()
