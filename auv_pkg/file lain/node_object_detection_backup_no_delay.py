#!/usr/bin/env python3
"""
Object Detection Node — Refactored
Fixes:
  1. Frame basi  → grab()+retrieve() buang buffer lama
  2. cap.read() blocking/freeze → CameraThread terpisah
  3. ROS image berat → CompressedImage JPEG
  4. Reconnect spam → cooldown timer
"""

import cv2
import time
import os
import threading

from ultralytics import YOLO
from auv_interfaces.msg import BoundingBox, ObjectDetection
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node


# ================= CONFIG =================
IMGSZ             = 640
CAM_PRIORITY      = [4, 5, 6, 7]
WARMUP_FRAMES     = 20
CONF_THRES        = 0.5
IMAGE_PUB_INTERVAL = 0.1   # 10 fps untuk publish image
JPEG_QUALITY      = 60     # 0–100; lebih rendah = lebih kecil & cepat
MAX_FAIL_COUNT    = 10
RECONNECT_COOLDOWN = 2.0   # detik antar percobaan reconnect
# ==========================================


# =============================================================================
# CAMERA THREAD — grab frame di background, tidak blocking main loop
# =============================================================================
class CameraThread:
    """
    Jalankan cap.read() di thread terpisah supaya main loop tidak pernah
    blocking karena tunggu frame dari kamera.
    """

    def __init__(self, cap: cv2.VideoCapture):
        self._cap   = cap
        self._ret   = False
        self._frame = None
        self._lock  = threading.Lock()
        self._stop  = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop:
            # Buang semua frame lama yang numpuk di buffer
            # dengan grab() non-blocking, baru retrieve() satu frame terbaru
            self._cap.grab()          # frame ke-(n-1) — dibuang
            ret, frame = self._cap.read()
            with self._lock:
                self._ret   = ret
                self._frame = frame

    def read(self):
        """Return (ret, frame_copy). frame bisa None kalau belum ada."""
        with self._lock:
            if self._frame is None:
                return False, None
            return self._ret, self._frame.copy()

    def stop(self):
        self._stop = True
        self._thread.join(timeout=2.0)

    def release(self):
        self.stop()
        self._cap.release()


# =============================================================================
# HELPERS
# =============================================================================
def open_camera(cam_ids: list) -> tuple[cv2.VideoCapture | None, int]:
    """
    Coba buka kamera dari daftar id secara berurutan.
    Return (cap, cam_id) dari kamera pertama yang berhasil di-read.
    Return (None, -1) kalau semua gagal.
    """
    for cam_id in cam_ids:
        cap = cv2.VideoCapture(cam_id)
        cap.set(cv2.CAP_PROP_FOURCC,       cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  IMGSZ)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMGSZ)
        cap.set(cv2.CAP_PROP_FPS,          30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # buffer sekecil mungkin

        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                return cap, cam_id

        cap.release()

    return None, -1


# =============================================================================
# NODE
# =============================================================================
class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('node_object_detection')

        # ----- Publishers -----
        self.obj_det_pub = self.create_publisher(ObjectDetection, 'object_detection', 10)

        # Pakai CompressedImage — jauh lebih ringan dari raw BGR8
        self.pub_image = self.create_publisher(
            CompressedImage, '/camera/main_cam/compressed', 10
        )

        # ----- Model -----
        self.model = YOLO(
            '/home/techsas/auv_ws/src/auv_pkg/pt/telkom12mei_best.engine',
            task='detect'
        )

        # ----- Misc -----
        self.bridge              = CvBridge()
        self.last_image_pub      = time.time()
        self._last_reconnect_at  = 0.0   # untuk cooldown reconnect
        self._fail_count         = 0

        # ----- CUDA check -----
        try:
            import torch
            self.use_cuda = torch.cuda.is_available()
        except ImportError:
            self.use_cuda = False

        if self.use_cuda:
            self.get_logger().info("✅ CUDA tersedia — pakai GPU.")
        else:
            self.get_logger().warn("⚠️  CUDA tidak tersedia — jalan di CPU (lambat).")

        # ----- Buka kamera -----
        cap, self.cam_id = open_camera(CAM_PRIORITY)
        if cap is None:
            raise RuntimeError(
                f"❌ Tidak bisa membuka kamera manapun dari {CAM_PRIORITY}. "
                "Pastikan kamera terhubung dan tidak dipakai proses lain."
            )

        self.get_logger().info(f"✅ Pakai kamera id={self.cam_id}")

        # Bungkus cap ke CameraThread
        self.cam_thread = CameraThread(cap)

        # ----- Warmup TensorRT -----
        self.get_logger().warn("🔥 Warming up TensorRT...")
        warmup_done = 0
        deadline    = time.time() + 10.0  # max 10 detik untuk warmup
        while warmup_done < WARMUP_FRAMES and time.time() < deadline:
            ret, frame = self.cam_thread.read()
            if ret and frame is not None:
                self.model.predict(frame, imgsz=IMGSZ, device=0, verbose=False)
                warmup_done += 1
            else:
                time.sleep(0.05)
        self.get_logger().warn(f"✅ Warmup selesai ({warmup_done} frame)")

    # =========================================================================
    # RECONNECT — dengan cooldown supaya tidak spam
    # =========================================================================
    def _try_reconnect(self):
        now = time.time()
        if now - self._last_reconnect_at < RECONNECT_COOLDOWN:
            # Masih dalam cooldown, skip dulu
            return

        self._last_reconnect_at = now
        self.get_logger().warn(
            f"⚠️  Kamera id={self.cam_id} hilang! "
            f"Mencoba reconnect dari {CAM_PRIORITY}..."
        )

        # Hentikan thread lama
        if self.cam_thread is not None:
            self.cam_thread.release()
            self.cam_thread = None

        cap, cam_id = open_camera(CAM_PRIORITY)

        if cap is not None:
            self.cam_id      = cam_id
            self.cam_thread  = CameraThread(cap)
            self._fail_count = 0
            self.get_logger().info(f"✅ Reconnect berhasil ke kamera id={self.cam_id}")
        else:
            self.get_logger().error(
                f"❌ Reconnect gagal. Tidak ada kamera dari {CAM_PRIORITY}."
            )

    # =========================================================================
    # SPIN ONCE — dipanggil setiap iterasi main loop
    # =========================================================================
    def spin_once(self):
        # Kalau belum ada thread (setelah reconnect gagal), coba lagi
        if self.cam_thread is None:
            self._try_reconnect()
            time.sleep(0.1)
            return

        ret, frame = self.cam_thread.read()

        # Deteksi kamera disconnect
        if not ret or frame is None:
            self._fail_count += 1
            self.get_logger().warn(
                f"Frame tidak diterima ({self._fail_count}/{MAX_FAIL_COUNT})"
            )
            if self._fail_count >= MAX_FAIL_COUNT:
                self._try_reconnect()
            time.sleep(0.05)
            return

        # Reset fail counter
        self._fail_count = 0

        # Titik tengah frame untuk referensi visual
        frame_cx = IMGSZ // 2
        frame_cy = 480  // 2
        cv2.circle(frame, (frame_cx, frame_cy), 3, (255, 0, 0), -1)

        # ===== Inferensi =====
        t0      = time.time()
        results = self.model.predict(
            frame,
            imgsz=IMGSZ,
            device=0,
            verbose=False
        )
        infer_time = time.time() - t0
        fps        = 1.0 / infer_time if infer_time > 0 else 0.0

        # ===== Build ROS Message + Gambar bounding box =====
        msg = ObjectDetection()

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls             = int(box.cls[0])
                label           = self.model.names[cls]

                # Gambar bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"{label} {conf:.2f}",
                    (x1, y1 - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )

                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f'({center_x}, {center_y})',
                    (center_x, center_y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1
                )

                # Isi ROS message
                bbox             = BoundingBox()
                bbox.class_name  = label
                bbox.probability = conf
                bbox.x_min       = x1
                bbox.y_min       = y1
                bbox.x_max       = x2
                bbox.y_max       = y2
                bbox.total_x     = x2 - x1

                msg.bounding_boxes.append(bbox)

        self.obj_det_pub.publish(msg)

        # FPS overlay
        cv2.putText(
            frame,
            f"FPS: {fps:.1f} | CAM: {self.cam_id}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
        )

        self.get_logger().info(
            f"[FPS] {fps:.1f} | CAM={self.cam_id} | "
            f"Deteksi {len(msg.bounding_boxes)} objek"
        )

        # ===== Publish image (throttled, CompressedImage JPEG) =====
        if t0 - self.last_image_pub >= IMAGE_PUB_INTERVAL:
            small_frame = cv2.resize(frame, (320, 240))

            # Encode ke JPEG — jauh lebih kecil dari raw BGR8
            ok, buf = cv2.imencode(
                '.jpg', small_frame,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )

            if ok:
                compressed_msg          = CompressedImage()
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.format   = "jpeg"
                compressed_msg.data     = buf.tobytes()
                self.pub_image.publish(compressed_msg)

            self.last_image_pub = t0

    # =========================================================================
    # CLEANUP
    # =========================================================================
    def destroy(self):
        if self.cam_thread is not None:
            self.cam_thread.release()
        cv2.destroyAllWindows()
        del self.model
        os._exit(0)  # anti TensorRT allocator crash


# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        while rclpy.ok():
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()