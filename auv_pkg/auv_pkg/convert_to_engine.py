from ultralytics import YOLO

# Load a YOLO11n PyTorch model
model = YOLO("SAUVC_FINAL_100.pt")

# Export the model to TensorRT
model.export(format="engine", imgsz=640, device=0, half=True)  # creates 'yolo11n.engine'

# Load the exported TensorRT model
trt_model = YOLO("SAUVC_FINAL_100.engine")

# Run inference
results = trt_model("https://ultralytics.com/images/bus.jpg")