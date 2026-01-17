#!/usr/bin/env python3
import cv2
import time
import torch
from ultralytics import YOLO

VIDEO = "/home/krbai/auv_ws/src/auv_pkg/auv_pkg/dataset_sauvc_objek.mp4"
fileModel = "SAUVC_FINAL_100.engine"   # .pt | .onnx | .engine
IMGSZ = 640
WARMUP = 20

# ukuran window (lebih kecil)
WIN_W = 640
WIN_H = 480

def main():
    print(f"\n▶ Benchmark {fileModel} | imgsz={IMGSZ}")

    model = YOLO(fileModel)

    cap = cv2.VideoCapture(VIDEO)
    assert cap.isOpened(), "❌ Cannot open video"

    # 🔥 Warmup
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
        results = model.predict(
            source=frame,
            imgsz=IMGSZ,
            device=0,
            verbose=False
        )
        if torch.cuda.is_available():
            torch.cuda.synchronize()

        infer_time = time.time() - t0
        total_time += infer_time
        frames += 1

        # ===== VISUALIZATION =====
        annotated = results[0].plot()

        # resize window
        annotated = cv2.resize(annotated, (WIN_W, WIN_H))

        fps = 1.0 / infer_time if infer_time > 0 else 0.0
        latency_ms = infer_time * 1000.0

        cv2.putText(
            annotated,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            annotated,
            f"Latency: {latency_ms:.2f} ms",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.imshow("YOLO Detection", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    avg_fps = frames / total_time
    avg_latency = (total_time / frames) * 1000

    print("====== RESULT ======")
    print(f"Frames processed : {frames}")
    print(f"Total infer time : {total_time:.2f} s")
    print(f"AVG FPS          : {avg_fps:.2f}")
    print(f"AVG latency      : {avg_latency:.2f} ms")

if __name__ == "__main__":
    main()