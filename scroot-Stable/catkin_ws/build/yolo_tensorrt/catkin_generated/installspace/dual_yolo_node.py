#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# Import custom messages
from yolo_tensorrt.msg import YoloDetection, YoloDetectionArray

# Suppress TensorRT warnings
TRT_LOGGER = trt.Logger(trt.Logger.INFO)

class HostDeviceMem:
    def __init__(self, host_mem, device_mem):
        self.host = host_mem
        self.device = device_mem

    def __str__(self):
        return "Host:\n" + str(self.host) + "\nDevice:\n" + str(self.device)

    def __repr__(self):
        return self.__str__()

class TRTInference:
    def __init__(self, engine_path):
        self.engine_path = engine_path
        self.runtime = trt.Runtime(TRT_LOGGER)
        self.engine = self.load_engine()
        self.context = self.engine.create_execution_context()
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers()

        # Get input and output shapes
        self.input_shape = self.engine.get_binding_shape(0)
        self.output_shape = self.engine.get_binding_shape(1)
        self.input_dtype = self.engine.get_binding_dtype(0) # Store input dtype
        
        rospy.loginfo(f"TensorRT Input Shape: {self.input_shape}")
        rospy.loginfo(f"TensorRT Output Shape: {self.output_shape}")

    def load_engine(self):
        with open(self.engine_path, "rb") as f:
            return self.runtime.deserialize_cuda_engine(f.read())

    def allocate_buffers(self):
        inputs = []
        outputs = []
        bindings = []
        stream = cuda.Stream()

        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding)) * self.engine.get_binding_dtype(binding).itemsize
            dtype_trt = self.engine.get_binding_dtype(binding)
            if dtype_trt == trt.DataType.FLOAT:
                dtype = np.float32
            elif dtype_trt == trt.DataType.HALF:
                dtype = np.float16
            elif dtype_trt == trt.DataType.INT8:
                dtype = np.int8
            elif dtype_trt == trt.DataType.INT32:
                dtype = np.int32
            elif dtype_trt == trt.DataType.BOOL:
                dtype = bool # Use Python's bool or np.bool_ if a numpy scalar is strictly needed
            else:
                raise TypeError(f"Unsupported TensorRT data type: {dtype_trt}")

            host_mem = cuda.pagelocked_empty(trt.volume(self.engine.get_binding_shape(binding)), dtype=dtype)
            device_mem = cuda.mem_alloc(size)

            bindings.append(int(device_mem))
            if self.engine.binding_is_input(binding):
                inputs.append(HostDeviceMem(host_mem, device_mem))
            else:
                outputs.append(HostDeviceMem(host_mem, device_mem))
        return inputs, outputs, bindings, stream

    def infer(self, input_image_np):
        # Copy input image to host buffer
        np.copyto(self.inputs[0].host, input_image_np.ravel())

        # Transfer input data to the GPU
        cuda.memcpy_htod_async(self.inputs[0].device, self.inputs[0].host, self.stream)

        # Run inference
        self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)

        # Transfer predictions from GPU to host
        for out in self.outputs:
            cuda.memcpy_dtoh_async(out.host, out.device, self.stream)

        # Synchronize the stream
        self.stream.synchronize()

        return [out.host for out in self.outputs]

class YOLONode:
    def __init__(self):
        rospy.init_node('dual_yolo_node', anonymous=True)

        self.bridge = CvBridge()
        self.trt_inference = TRTInference(engine_path="/home/jetson/yolov8n_fp16.engine")

        self.input_image_topic = "/usb_cam/image_raw"
        self.annotated_image_topic = "/AI/annotated_image"
        self.detections_topic = "/AI/detections"

        self.image_sub = rospy.Subscriber(self.input_image_topic, Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher(self.annotated_image_topic, Image, queue_size=1)
        self.detection_pub = rospy.Publisher(self.detections_topic, YoloDetectionArray, queue_size=1)

        self.conf_threshold = 0.25
        self.iou_threshold = 0.45
        self.input_width = 640
        self.input_height = 640
        self.num_classes = 80 # Assuming COCO dataset

        self.class_names = [
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
            "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
            "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
            "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
            "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
            "hair drier", "toothbrush"
        ]
        rospy.loginfo("YOLOv8 TensorRT Node Initialized.")

    def preprocess(self, cv_image):
        # Resize image to input shape
        resized_image = cv2.resize(cv_image, (self.input_width, self.input_height))
        
        # Convert to RGB (if not already)
        if len(resized_image.shape) == 3 and resized_image.shape[2] == 3:
            rgb_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
        else:
            rgb_image = resized_image # Assume it's already RGB or grayscale to be handled by TRT

        # Normalize to [0, 1] and transpose to CHW format
        input_tensor = rgb_image.astype(np.float32) / 255.0
        input_tensor = np.transpose(input_tensor, (2, 0, 1)) # HWC to CHW
        input_tensor = np.expand_dims(input_tensor, axis=0) # Add batch dimension

        # Convert to the engine's expected input dtype
        if self.trt_inference.input_dtype == trt.DataType.HALF:
            input_tensor = input_tensor.astype(np.float16)
        elif self.trt_inference.input_dtype == trt.DataType.FLOAT:
            input_tensor = input_tensor.astype(np.float32)
        # Add other dtypes if necessary
        
        return input_tensor

    def postprocess(self, output, original_image_shape):
        # Output is (1, 84, 8400) - transpose to (1, 8400, 84)
        output = output[0].reshape(self.trt_inference.output_shape)
        output = np.transpose(output, (0, 2, 1)) # (1, 84, 8400) -> (1, 8400, 84)
        output = output[0] # Remove batch dimension (8400, 84)

        boxes = output[:, :4]
        scores = output[:, 4]
        class_scores = output[:, 5:]

        # Apply confidence threshold
        conf_mask = scores > self.conf_threshold
        boxes = boxes[conf_mask]
        scores = scores[conf_mask]
        class_scores = class_scores[conf_mask]

        if len(boxes) == 0:
            return []

        # Convert boxes from xywh to xyxy
        boxes_xyxy = np.copy(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2  # x1
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2  # y1
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2  # x2
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2  # y2

        # Scale bounding boxes to original image size
        scale_x = original_image_shape[1] / self.input_width
        scale_y = original_image_shape[0] / self.input_height
        boxes_xyxy[:, 0] *= scale_x
        boxes_xyxy[:, 1] *= scale_y
        boxes_xyxy[:, 2] *= scale_x
        boxes_xyxy[:, 3] *= scale_y

        # Perform NMS
        class_ids = np.argmax(class_scores, axis=1)
        
        # Prepare for NMS: combine boxes, scores, and class_ids
        # NMS requires boxes in (x1, y1, x2, y2) format
        
        # Filter out boxes with low class scores
        final_detections = []
        for i in range(len(boxes_xyxy)):
            class_id = class_ids[i]
            class_score = class_scores[i, class_id]
            overall_score = scores[i] * class_score # Combine objectness score with class score

            if overall_score > self.conf_threshold:
                final_detections.append([boxes_xyxy[i, 0], boxes_xyxy[i, 1], boxes_xyxy[i, 2], boxes_xyxy[i, 3], overall_score, class_id])
        
        if not final_detections:
            return []

        final_detections = np.array(final_detections)
        
        # Apply NMS per class
        unique_classes = np.unique(final_detections[:, 5])
        nms_detections = []

        for cls_id in unique_classes:
            cls_mask = final_detections[:, 5] == cls_id
            cls_boxes = final_detections[cls_mask, :4]
            cls_scores = final_detections[cls_mask, 4]

            indices = cv2.dnn.NMSBoxes(cls_boxes.tolist(), cls_scores.tolist(), self.conf_threshold, self.iou_threshold)
            if len(indices) > 0:
                for i in indices.flatten():
                    nms_detections.append(final_detections[cls_mask][i])

        return nms_detections

    def publish_msgs(self, detections, original_image, header):
        # Publish annotated image
        annotated_image_msg = self.bridge.cv2_to_imgmsg(original_image, "bgr8")
        annotated_image_msg.header = header
        self.image_pub.publish(annotated_image_msg)

        # Publish detections
        detection_array_msg = YoloDetectionArray()
        detection_array_msg.header = header

        for det in detections:
            x1, y1, x2, y2, score, class_id = det
            yolo_detection = YoloDetection()
            yolo_detection.x1 = float(x1)
            yolo_detection.y1 = float(y1)
            yolo_detection.x2 = float(x2)
            yolo_detection.y2 = float(y2)
            yolo_detection.score = float(score)
            yolo_detection.class_id = int(class_id)
            yolo_detection.class_name = self.class_names[int(class_id)]
            detection_array_msg.detections.append(yolo_detection)
        
        self.detection_pub.publish(detection_array_msg)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        original_image_shape = cv_image.shape # H, W, C

        # Preprocess
        input_tensor = self.preprocess(cv_image)

        # Inference
        outputs = self.trt_inference.infer(input_tensor)

        # Postprocess
        detections = self.postprocess(outputs, original_image_shape)

        # Draw bounding boxes on the original image
        annotated_image = cv_image.copy()
        for det in detections:
            x1, y1, x2, y2, score, class_id = det
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            class_name = self.class_names[int(class_id)]
            
            color = (0, 255, 0) # Green
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated_image, f"{class_name} {score:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Publish messages
        self.publish_msgs(detections, annotated_image, msg.header)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YOLONode()
        node.run()
    except rospy.ROSInterruptException:
        pass
