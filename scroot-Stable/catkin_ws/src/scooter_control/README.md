## 5. Jetson Optimization Guide

To achieve real-time performance on the Jetson platform, especially with two YOLO models, you must perform several optimizations. The provided Python code is functional but will be too slow without these steps.

### 1. Set Max Performance Mode (NVPModel)

Your Jetson has different power profiles. For autonomous operation, you must enable the maximum performance mode to unlock the full potential of the CPU and GPU.

```bash
# Check the current mode
sudo nvpmodel -q

# Set to MAXN mode (maximum performance)
sudo nvpmodel -m 0
```

It's crucial to do this before running your ROS nodes. You can add this command to a startup script.

### 2. Build and Use TensorRT Engines

TensorRT is NVIDIA's high-performance deep learning inference optimizer and runtime. It can provide a 5-10x speedup over running models directly from PyTorch or TensorFlow. You need to convert your `.pt` YOLO models into `.engine` files.

**Step A: Export to ONNX**

First, export your trained YOLOv8 models to the ONNX (Open Neural Network Exchange) format.

```bash
# From your training machine or the Jetson
pip install ultralytics
yolo export model=yolov8n.pt format=onnx opset=12
yolo export model=yolov8s.pt format=onnx opset=12
```
This will create `yolov8n.onnx` and `yolov8s.onnx`.

**Step B: Build the Engine on the Jetson**

Now, use the `trtexec` command on the Jetson to convert the ONNX file into a TensorRT engine. This process is device-specific and must be done on the target hardware.

```bash
# Example for YOLOv8n with FP16 precision
/usr/src/tensorrt/bin/trtexec --onnx=yolov8n.onnx \
                             --saveEngine=yolov8n_fp16.engine \
                             --fp16

# Example for YOLOv8s with FP16 precision
/usr/src/tensorrt/bin/trtexec --onnx=yolov8s.onnx \
                             --saveEngine=yolov8s_fp16.engine \
                             --fp16
```
*   `--fp16`: Using FP16 precision provides a significant speedup with minimal accuracy loss, which is ideal for most robotics applications.
*   You can also add `--workspace=<size>` to give TensorRT more memory for optimization.

**Step C: Integrate into ROS Node**

The `dual_yolo_node.py` must be modified to use these engines. You cannot use the simple `YOLO('model.pt')` call. You will need a Python library like `nvidia-tensorrt` or a C++ wrapper. A common approach is to use a helper class that handles:
1.  Loading the `.engine` file.
2.  Creating an execution context.
3.  Allocating GPU memory for inputs and outputs.
4.  Pre-processing the input image (resizing, normalizing, converting to NCHW format).
5.  Running the inference.
6.  Post-processing the output (parsing bounding boxes, applying NMS).

Libraries like `torch2trt` or community-provided wrappers for YOLO on TensorRT can simplify this process.

### 3. Image Resizing and Pre-processing

Running YOLO on a full-resolution camera stream (e.g., 1280x720) is inefficient.
*   **Resize in the Node**: Before sending the image to the model, resize it to the size the model was trained on (e.g., 640x640 or 320x320). This dramatically reduces latency. The `dual_yolo_node.py` should perform this `cv2.resize` operation.
*   **Choose Ideal Stride/Size**: For the YOLOv8n (low latency) model, consider using a smaller input size like 320x320. For the YOLOv8s (accuracy) model, 640x640 might be a good trade-off. Ensure the input size is a multiple of the model's max stride (usually 32).

### 4. CPU/GPU Threading Separation

The `dual_yolo_node.py` performs multiple tasks: receiving images, running two models, and fusing results. A single-threaded Python script will be slow.

**Recommended Approach:**
Use Python's `threading` library to offload the inference tasks.

```python
# Inside DualYoloNode class
import threading

def image_callback(self, msg):
    # ... cv_bridge conversion ...

    # --- YOLOv8n (always runs in its own thread) ---
    thread_n = threading.Thread(target=self.run_inference_n, args=(cv_image, msg.header))
    thread_n.start()

    # --- YOLOv8s (runs periodically) ---
    if self.frame_counter % self.yolo_s_frame_interval == 0:
      # You can run this in the main thread or another thread
      # For simplicity, let's run it here and fuse with the last 'n' result
      # A more complex setup would use thread-safe queues to pass results around
      ...

def run_inference_n(self, image, header):
    # This function runs in a separate thread
    results_n = self.model_n.predict(image)
    detections_n = self._results_to_detections(results_n)
    # If not fusing this frame, publish directly
    if self.frame_counter % self.yolo_s_frame_interval != 0:
        self._publish_detections(detections_n, header, "yolov8n")
```

This is a simplified example. A robust implementation would use thread-safe queues (`Queue.Queue` in Python 2) to pass images to worker threads and collect results, preventing race conditions and ensuring smooth data flow. The main ROS thread should only handle subscribing and publishing, while worker threads handle the heavy computation (inference).
