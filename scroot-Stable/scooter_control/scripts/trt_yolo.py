#!/usr/bin/env python3
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np
import cv2
import time

class TRT_YOLO:
    def __init__(self, engine_path, input_size=(640, 640)):
        self.input_h, self.input_w = input_size

        trt_logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(trt_logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        self.input_binding = self.engine.get_binding_index("images")
        self.output_binding = self.engine.get_binding_index("output0")

        self.input_shape = self.engine.get_binding_shape(self.input_binding)
        self.output_shape = self.engine.get_binding_shape(self.output_binding)

        self.input_size_bytes = np.prod(self.input_shape) * np.float32().nbytes
        self.output_size_bytes = np.prod(self.output_shape) * np.float32().nbytes

        self.d_input = cuda.mem_alloc(self.input_size_bytes)
        self.d_output = cuda.mem_alloc(self.output_size_bytes)

        self.h_output = cuda.pagelocked_empty(np.prod(self.output_shape), dtype=np.float32)

    def preprocess(self, img):
        resized = cv2.resize(img, (self.input_w, self.input_h))
        rgb = resized[:, :, ::-1].astype(np.float32) / 255.0
        transposed = np.transpose(rgb, (2, 0, 1))
        return np.expand_dims(transposed, axis=0).astype(np.float32)

    def infer(self, img):
        blob = self.preprocess(img)

        cuda.memcpy_htod(self.d_input, blob)

        self.context.execute_v2([int(self.d_input), int(self.d_output)])

        cuda.memcpy_dtoh(self.h_output, self.d_output)

        return self.h_output.reshape(self.output_shape)

