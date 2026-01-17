#!/usr/bin/env python3
import cv2
import time
import numpy as np
import onnxruntime as ort

# ================== CONFIG ==================
IMGSZ = 640
CAM_ID = 0
BENCH_TIME = 30.0
WARMUP_FRAMES = 20
CONF_THRES = 0.25
IOU_THRES = 0.45

# COCO 80 classes
CLS_NAMES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","couch","potted plant","bed","dining table","toilet",
    "tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush"
]

# ================== PREPROCESS ==================
def preprocess(frame):
    img = cv2.resize(frame, (IMGSZ, IMGSZ))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))   # CHW
    img = np.expand_dims(img, axis=0)    # NCHW
    return img

# ================== POSTPROCESS ==================
def postprocess(outputs, frame_shape):
    preds = outputs[0][0]          # (84, 8400)
    preds = preds.transpose(1, 0)  # (8400, 84)

    h, w = frame_shape[:2]
    boxes, scores, class_ids = [], [], []

    for p in preds:
        obj_conf = float(p[4])
        if obj_conf < CONF_THRES:
            continue

        cls_scores = p[5:]
        cls_id = int(np.argmax(cls_scores))
        score = obj_conf * float(cls_scores[cls_id])
        if score < CONF_THRES:
            continue

        cx, cy, bw, bh = map(float, p[:4])

        # convert to pixel
        x1 = int((cx - bw / 2) * w)
        y1 = int((cy - bh / 2) * h)
        x2 = int((cx + bw / 2) * w)
        y2 = int((cy + bh / 2) * h)

        # ===== CLAMP (PENTING!) =====
        x1 = max(0, min(x1, w - 1))
        y1 = max(0, min(y1, h - 1))
        x2 = max(0, min(x2, w - 1))
        y2 = max(0, min(y2, h - 1))

        bw = x2 - x1
        bh = y2 - y1

        # ===== SKIP INVALID BOX =====
        if bw <= 1 or bh <= 1:
            continue

        boxes.append([x1, y1, bw, bh])
        scores.append(float(score))
        class_ids.append(cls_id)

    # ===== GUARD =====
    if len(boxes) == 0:
        return []

    indices = cv2.dnn.NMSBoxes(
        boxes, scores,
        CONF_THRES, IOU_THRES
    )

    if len(indices) == 0:
        return []

    results = []
    for i in indices.flatten():
        results.append((boxes[i], scores[i], class_ids[i]))

    return results


# ================== MAIN ==================
def main():
    print("▶ Benchmark YOLO11n.onnx | ONNX Runtime CUDA | imgsz=640")

    # ONNX Runtime Session (NO affinity warning)
    so = ort.SessionOptions()
    so.intra_op_num_threads = 1
    so.inter_op_num_threads = 1

    providers = [
        ("CUDAExecutionProvider", {
            "cudnn_conv_algo_search": "EXHAUSTIVE"
        }),
        "CPUExecutionProvider"
    ]

    sess = ort.InferenceSession(
        "yolo11n.onnx",
        sess_options=so,
        providers=providers
    )

    input_name = sess.get_inputs()[0].name
    output_names = [o.name for o in sess.get_outputs()]

    cap = cv2.VideoCapture(CAM_ID, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMGSZ)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMGSZ)
    cap.set(cv2.CAP_PROP_FPS, 60)

    if not cap.isOpened():
        raise RuntimeError("❌ Cannot open camera")

    # ========== WARMUP ==========
    print("🔥 Warming up...")
    for _ in range(WARMUP_FRAMES):
        ret, frame = cap.read()
        if ret:
            blob = preprocess(frame)
            sess.run(output_names, {input_name: blob})

    print("⏱️ Start 30s benchmark (press 'q' to exit early)")

    frames = 0
    total_time = 0.0
    t_start = time.time()

    while time.time() - t_start < BENCH_TIME:
        ret, frame = cap.read()
        if not ret:
            continue

        blob = preprocess(frame)

        # ===== INFERENCE ONLY =====
        t0 = time.time()
        outputs = sess.run(output_names, {input_name: blob})
        infer_time = time.time() - t0

        total_time += infer_time
        frames += 1

        # ===== POST + DRAW (OUTSIDE TIMING) =====
        results = postprocess(outputs, frame.shape)

        for box, score, cls_id in results:
            x, y, w, h = box
            label = f"{CLS_NAMES[cls_id]} {score:.2f}"

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                frame, label, (x, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 0), 2
            )

        vis_fps = 1.0 / infer_time if infer_time > 0 else 0
        cv2.putText(
            frame, f"FPS: {vis_fps:.1f}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (0, 0, 255), 2
        )

        cv2.imshow("YOLO11n ONNX - Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    avg_fps = frames / total_time
    avg_latency = (total_time / frames) * 1000

    print("\n====== BENCH RESULT (ONNX) ======")
    print(f"Frames processed : {frames}")
    print(f"Total infer time : {total_time:.2f} s")
    print(f"AVG FPS          : {avg_fps:.2f}")
    print(f"AVG latency      : {avg_latency:.2f} ms")

if __name__ == "__main__":
    main()
