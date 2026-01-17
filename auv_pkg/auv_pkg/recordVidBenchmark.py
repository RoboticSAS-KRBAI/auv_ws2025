#!/usr/bin/env python3
import cv2
import time

# ===============================
# CONFIG
# ===============================
CAM_ID = 0
DURATION_SEC = 30
WIDTH = 640
HEIGHT = 480
FPS = 30
OUTPUT = "input_30sv2.mp4"

def main():
    cap = cv2.VideoCapture(CAM_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    if not cap.isOpened():
        print("❌ Cannot open camera")
        return

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(OUTPUT, fourcc, FPS, (WIDTH, HEIGHT))

    print("🎥 Recording started (30 seconds)...")
    start = time.time()
    frames = 0

    while time.time() - start < DURATION_SEC:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Frame drop")
            continue

        writer.write(frame)
        frames += 1

    elapsed = time.time() - start

    cap.release()
    writer.release()

    print("✅ Recording finished")
    print(f"Saved as: {OUTPUT}")
    print(f"Frames captured: {frames}")
    print(f"Actual duration: {elapsed:.2f} s")
    print(f"Actual FPS: {frames / elapsed:.2f}")

if __name__ == "__main__":
    main()
