#!/usr/bin/env python3
import cv2
import time
import os

from ultralytics import YOLO
from auv_interfaces.msg import BoundingBox, ObjectDetection

import rclpy
from rclpy.node import Node


# ================= CONFIG =================
IMGSZ = 640
CAM_ID = 0
WARMUP_FRAMES = 20
CONF_THRES = 0.55
# ==========================================


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('node_object_detection')

        self.obj_det_pub = self.create_publisher(
            ObjectDetection, 'object_detection', 10
        )

        # ===== Load TensorRT Engine =====
        self.model = YOLO(
            '/home/techsas/auv_ws/src/auv_pkg/pt/full-itb-17mar26(notclean).engine',
            task='detect'
        )

        # ===== Check CUDA =====
        self.use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
        if self.use_cuda:
            self.get_logger().info("✅ CUDA is available. Using GPU acceleration.")
        else:
            self.get_logger().warn("⚠️ CUDA is NOT available. Running on CPU, which may be slow.")

        # ===== Camera =====
        self.cap = cv2.VideoCapture(CAM_ID)
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMGSZ)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMGSZ)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError("❌ Cannot open camera")

        # ===== Warmup =====
        self.get_logger().warn("🔥 Warming up TensorRT...")
        for _ in range(WARMUP_FRAMES):
            ret, frame = self.cap.read()
            if ret:
                self.model.predict(
                    frame,
                    imgsz=IMGSZ,
                    device=0,
                    verbose=False
                )
        self.get_logger().warn("✅ Warmup done")

    def spin_once(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        frame_cx, frame_cy = IMGSZ // 2, 480 // 2
        cv2.circle(frame, (frame_cx, frame_cy), 3, (255, 0, 0), -1)

        # ===== Inference ONLY =====
        t0 = time.time()
        results = self.model.predict(
            frame,
            imgsz=IMGSZ,
            device=0,
            verbose=False
        )
        infer_time = time.time() - t0
        fps = 1.0 / infer_time if infer_time > 0 else 0.0

        # ===== Build ROS Message + Draw =====
        msg = ObjectDetection()

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                # ---- Draw Bounding Box ----
                cv2.rectangle(
                    frame, (x1, y1), (x2, y2),
                    (0, 255, 0), 2
                )
                cv2.putText(
                    frame,
                    f"{label} {conf:.2f}",
                    (x1, y1 - 7),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                center_x_vis = (x1 + x2) // 2
                center_y_vis = (y1 + y2) // 2
                cv2.circle(frame, (center_x_vis, center_y_vis), 3, (0, 0, 255), -1)
                cv2.putText(frame, f'({center_x_vis}, {center_y_vis})',
                            (center_x_vis, center_y_vis + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)


                # ---- ROS Message ----
                bbox = BoundingBox()
                bbox.class_name = label
                bbox.probability = conf
                bbox.x_min = x1
                bbox.y_min = y1
                bbox.x_max = x2
                bbox.y_max = y2
                bbox.total_x = x2 - x1

                msg.bounding_boxes.append(bbox)

        self.obj_det_pub.publish(msg)

        # ===== Visual (optional) =====
        cv2.putText(
            frame, f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (0, 0, 255), 2
        )
        cv2.imshow("YOLO TensorRT ROS2", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()
        del self.model
        os._exit(0)  # anti TensorRT allocator crash


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        while rclpy.ok():
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1/30)  # lock ke 30 FPS hanya jika inferensi terlalu cepat (jangan gunakan jika di live camera)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()