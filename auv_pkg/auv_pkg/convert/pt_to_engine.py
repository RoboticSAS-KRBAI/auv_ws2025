#!/usr/bin/env python3
from ultralytics import YOLO

# Load a YOLO11n PyTorch model
model = YOLO("/home/techsas/auv_ws/src/auv_pkg/pt/telkom12mei_best.pt")

# Export the model to TensorRT
model.export(format="engine", imgsz=640, device=0, half=True) #half=False 30+fps #half=True bisa sampe 50+fps # creates 'yolo11n.engine'

# Load the exported TensorRT model
trt_model = YOLO("/home/techsas/auv_ws/src/auv_pkg/pt/telkom12mei_best.engine")