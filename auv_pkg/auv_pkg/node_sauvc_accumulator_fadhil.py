#!/usr/bin/env python3
"""
SAUVC Accumulator Node
=======================
Terima raw ObjectDetection → filter berdasarkan active_target dari Guidance
→ publish ObjectDifference (x_diff, bbox_size, dll.) ke Guidance.

Perbaikan dari versi sebelumnya:
  1. Pilih bbox confidence tertinggi, bukan yang pertama ditemukan.
     (jika ada 2 flare warna sama dalam frame, ambil yang paling yakin)
  2. Tambah field bounding_box_size ke output → dipakai Guidance untuk
     mendeteksi "sudah dekat" saat RAMMING tanpa harus hard-code waktu.
  3. Tambah field is_target (konsisten dengan ObjectDifference msg lama).
  4. FRAME_CENTER_X dijadikan konstanta atas, tidak hard-code di dalam fungsi.
  5. Log saat active_target berubah (debug lebih mudah).
"""

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import ObjectDetection, ObjectDifference
from std_msgs.msg import String

# Harus sesuai dengan resolusi kamera yang diset di object detection node
FRAME_CENTER_X = 320   # 640 // 2


class SubAccumulator(Node):
    def __init__(self):
        super().__init__('accumulator_subscriber')

        # Target aktif dikirim oleh Guidance
        self.active_target = "orange_flare"

        # Publisher
        self.pub_object_difference = self.create_publisher(
            ObjectDifference, 'object_difference', 10)

        # Subscribers
        self.create_subscription(
            ObjectDetection, 'object_detection',
            self.object_detection_callback, 10)

        self.create_subscription(
            String, 'active_target',
            self.active_target_callback, 10)

        self.get_logger().info("✅ Accumulator Started")

    # ── Callbacks ───────────────────────────────────────────

    def active_target_callback(self, msg: String):
        if msg.data != self.active_target:
            self.get_logger().info(f"Active target → {msg.data}")
        self.active_target = msg.data

    def object_detection_callback(self, data: ObjectDetection):
        diff = ObjectDifference()
        diff.object_type       = "None"
        diff.x_difference      = 0
        diff.is_target         = False
        diff.bounding_box_size = 0

        # ── Cari bbox active_target dengan confidence tertinggi ──
        # BUG LAMA: ambil yang pertama (break setelah match pertama).
        # Jika ada 2 objek dengan kelas sama, bisa saja yang pertama
        # bukan yang terbaik (confidence rendah / posisi buruk).
        # FIX: iterasi semua bbox, pilih confidence tertinggi.
        best_bbox = None
        best_conf = -1.0

        for bbox in data.bounding_boxes:
            if bbox.class_name == self.active_target:
                if bbox.probability > best_conf:
                    best_conf = bbox.probability
                    best_bbox = bbox

        if best_bbox is not None:
            cx = (best_bbox.x_min + best_bbox.x_max) // 2

            diff.object_type       = best_bbox.class_name
            diff.x_difference      = cx - FRAME_CENTER_X
            diff.is_target         = True
            # lebar bbox dalam pixel → indikator seberapa dekat objek ke kamera
            diff.bounding_box_size = best_bbox.x_max - best_bbox.x_min

        self.pub_object_difference.publish(diff)


# ════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = SubAccumulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()