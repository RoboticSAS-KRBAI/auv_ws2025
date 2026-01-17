#!/usr/bin/env python3
import cv2
import time
import torch
from ultralytics import YOLO

VIDEO = "input_30sv2.mp4"
fileModel = "yolo11n.onnx"   # .pt | .onnx | .engine
IMGSZ = 640
WARMUP = 20

def main():
    print(f"\n▶ Benchmark {fileModel} | imgsz={IMGSZ}")

    model = YOLO(fileModel)

    cap = cv2.VideoCapture(VIDEO)
    assert cap.isOpened(), "❌ Cannot open video"

    # 🔥 Warmup (PENTING!)
    for _ in range(WARMUP):
        ret, frame = cap.read()
        if not ret:
            break
        model.predict(
            source=frame,
            imgsz=IMGSZ,
            device=0,
            verbose=False
        )
        if torch.cuda.is_available():
            torch.cuda.synchronize()

    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    frames = 0
    total_time = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t0 = time.time()
        model.predict(
            source=frame,
            imgsz=IMGSZ,
            device=0,
            verbose=False
        )
        if torch.cuda.is_available():
            torch.cuda.synchronize()

        total_time += time.time() - t0
        frames += 1

    cap.release()

    avg_fps = frames / total_time
    avg_latency = (total_time / frames) * 1000

    print("====== RESULT ======")
    print(f"Frames processed : {frames}")
    print(f"Total infer time : {total_time:.2f} s")
    print(f"AVG FPS          : {avg_fps:.2f}")
    print(f"AVG latency      : {avg_latency:.2f} ms")

if __name__ == "__main__":
    main()
