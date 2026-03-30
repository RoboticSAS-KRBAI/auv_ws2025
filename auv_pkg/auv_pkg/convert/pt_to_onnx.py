#!/usr/bin/env python3
from ultralytics import YOLO

# Load the YOLO26 model
model = YOLO("/home/techsas/auv_ws/src/auv_pkg/pt/yolo26_kolam_telkom.pt")

# Export the model to ONNX format
model.export(format="onnx")  # creates 'yolo26n.onnx'

# Load the exported ONNX model
onnx_model = YOLO("/home/techsas/auv_ws/src/auv_pkg/pt/yolo26_kolam_telkom.onnx")