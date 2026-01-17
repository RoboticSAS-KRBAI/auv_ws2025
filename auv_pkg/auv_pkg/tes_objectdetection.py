#!/usr/bin/env python3
import cv2
import time
import os
from ultralytics import YOLO

IMGSZ = 640
CAM_ID = 0
BENCH_TIME = 30.0   # detik
WARMUP_FRAMES = 20

def main():
    print("▶ Benchmark TensorRT (.engine) | Camera + Display | imgsz=640")

    model = YOLO("yolo11n.engine", task="detect")

    cap = cv2.VideoCapture(CAM_ID, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMGSZ)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMGSZ)
    cap.set(cv2.CAP_PROP_FPS, 60)

    if not cap.isOpened():
        raise RuntimeError("❌ Cannot open camera")

    # ===== Warmup =====
    print("🔥 Warming up TensorRT...")
    for _ in range(WARMUP_FRAMES):
        ret, frame = cap.read()
        if ret:
            model.predict(
                source=frame,
                imgsz=IMGSZ,
                device=0,
                verbose=False
            )

    print("⏱️ Start 30s benchmark (press 'q' to exit early)")

    frames = 0
    total_time = 0.0
    t_start = time.time()

    while True:
        if time.time() - t_start >= BENCH_TIME:
            break

        ret, frame = cap.read()
        if not ret:
            continue

        # ===== Inference timing (ONLY THIS) =====
        t0 = time.time()
        results = model.predict(
            source=frame,
            imgsz=IMGSZ,
            device=0,
            verbose=False
        )
        infer_time = time.time() - t0

        total_time += infer_time
        frames += 1

        # ===== Draw result (OUTSIDE timing) =====
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = f"{model.names[cls]} {conf:.2f}"

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )

        # Live FPS (visual only)
        vis_fps = 1.0 / infer_time if infer_time > 0 else 0
        cv2.putText(
            frame, f"FPS: {vis_fps:.1f}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (0, 0, 255), 2
        )

        cv2.imshow("TensorRT YOLO11n - Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    del model

    avg_fps = frames / total_time
    avg_latency = (total_time / frames) * 1000

    print("\n====== BENCH RESULT ======")
    print(f"Frames processed : {frames}")
    print(f"Total infer time : {total_time:.2f} s")
    print(f"AVG FPS          : {avg_fps:.2f}")
    print(f"AVG latency      : {avg_latency:.2f} ms")

    # 🚑 anti crash allocator
    os._exit(0)

if __name__ == "__main__":
    main()
