#!/usr/bin/env python3
from ultralytics import YOLO

# Load a YOLO11n PyTorch model
model = YOLO("src/auv_pkg/pt/full-itb-17mar26(notclean).pt")

# Export the model to TensorRT
model.export(format="engine", imgsz=640, device=0, half=False) #half=False 30+fps #half=True bisa sampe 50+fps # creates 'yolo11n.engine'

# Load the exported TensorRT model
trt_model = YOLO("src/auv_pkg/pt/full-itb-17mar26(notclean).engine")
