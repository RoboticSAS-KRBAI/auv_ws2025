#!/usr/bin/env python3
"""
TEST FLAG 1 — Dodge Orange Flare
==================================
Jalankan file ini untuk test Flag 1 saja secara isolated.

Alur:
  INIT → SEARCHING → CENTERING → SWAYING → FORWARD_PASS → DONE (surface)

Setelah FORWARD_PASS selesai, robot langsung surfacing.
Tidak ada transisi ke Flag 2/3/4.

Cara pakai:
  ros2 run auv_pkg node_test_flag1
  (atau: python3 node_test_flag1.py)

Yang perlu jalan bersamaan:
  - node_object_detection
  - node_sauvc_accumulator_fadhil
  - node kontrol motor (teensy)
  - node_map_sauvc (opsional, hanya untuk visualisasi)
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDifference

# ── Konfigurasi ─────────────────────────────────────────────────────────────
CAMERA_TOL          = 30      # pixel tolerance centering
CENTERING_HOLD_SEC  = 3.0     # detik harus di tengah sebelum sway
SWAYING_SEC         = 2.0     # durasi sway (detik)
FORWARD_PASS_SEC    = 3.0     # durasi maju setelah sway (detik)
SEARCHING_TIMEOUT   = 8.0     # timeout searching sebelum skip (detik)
# ────────────────────────────────────────────────────────────────────────────


class Flag1Node(Node):
    def __init__(self):
        super().__init__('test_flag1')

        self.sub_state              = "INIT"
        self.state_start_time       = time.time()
        self.within_tolerance_start = None   # timer centering independen

        self.x_diff       = 0
        self.detected_obj = "None"

        self.flare_position_memory = ""   # "left" | "right"

        # ── Publishers ──────────────────────────────────────
        self.pub_multi_pid     = self.create_publisher(MultiPID, "pid",           10)
        self.pub_set_point     = self.create_publisher(SetPoint, "set_point",     10)
        self.pub_status        = self.create_publisher(String,   "status",        10)
        self.pub_active_target = self.create_publisher(String,   "active_target", 10)
        self.pub_boost         = self.create_publisher(Float32,  "boost",         10)

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(
            ObjectDifference, 'object_difference', self.obj_diff_cb, 10)

        # ── SetPoint & PID awal ─────────────────────────────
        self.set_point       = SetPoint()
        self.set_point.yaw   = 0.0
        self.set_point.pitch = 0.0
        self.set_point.roll  = 1.2
        self.set_point.depth = -0.11

        self.multi_pid_msg          = MultiPID()
        pid_yaw                     = PID(); pid_yaw.kp = 4.5;    pid_yaw.ki = 0.0;  pid_yaw.kd = 0.3
        pid_pitch                   = PID(); pid_pitch.kp = 15.0; pid_pitch.ki = 0.0; pid_pitch.kd = 2.6
        pid_roll                    = PID(); pid_roll.kp = 2.5;   pid_roll.ki = 0.0;  pid_roll.kd = 0.4
        pid_depth                   = PID(); pid_depth.kp = 1350.0; pid_depth.ki = 0.0; pid_depth.kd = 215.0
        pid_camera                  = PID(); pid_camera.kp = 0.5; pid_camera.ki = 0.0; pid_camera.kd = 0.0
        self.multi_pid_msg.pid_yaw    = pid_yaw
        self.multi_pid_msg.pid_pitch  = pid_pitch
        self.multi_pid_msg.pid_roll   = pid_roll
        self.multi_pid_msg.pid_depth  = pid_depth
        self.multi_pid_msg.pid_camera = pid_camera

        self.pub_multi_pid.publish(self.multi_pid_msg)
        self.pub_set_point.publish(self.set_point)
        self._pub_boost(350.0)

        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("🟢 [FLAG 1 TEST] Node started")

    # ── Callbacks ───────────────────────────────────────────
    def obj_diff_cb(self, msg: ObjectDifference):
        self.detected_obj = msg.object_type
        self.x_diff       = msg.x_difference

    # ── Helpers ─────────────────────────────────────────────
    def _pub_status(self, s: str):
        msg = String(); msg.data = s
        self.pub_status.publish(msg)

    def _pub_boost(self, v: float):
        msg = Float32(); msg.data = v
        self.pub_boost.publish(msg)

    def _elapsed(self) -> float:
        return time.time() - self.state_start_time

    def _change_state(self, new_state: str):
        self.sub_state              = new_state
        self.state_start_time       = time.time()
        self.within_tolerance_start = None
        self.get_logger().info(f"  ──► {new_state}")

    def _surface(self):
        """Robot selesai — naik ke permukaan."""
        self.get_logger().info("✅ [FLAG 1] Selesai! Surfacing...")
        self.set_point.depth = 0.0
        self.pub_set_point.publish(self.set_point)
        self._change_state("DONE")

    # ── Main loop ───────────────────────────────────────────
    def loop(self):
        self.pub_multi_pid.publish(self.multi_pid_msg)
        self.pub_set_point.publish(self.set_point)
        self.pub_active_target.publish(String(data="orange_flare"))

        # ── INIT ──────────────────────────────────────────
        if self.sub_state == "INIT":
            self.get_logger().info("🔍 [FLAG 1] INIT → SEARCHING")
            self._change_state("SEARCHING")

        # ── SEARCHING ─────────────────────────────────────
        # Maju perlahan (camera) mencari orange flare.
        # Timeout SEARCHING_TIMEOUT detik → anggap tidak ada flare, langsung surface.
        elif self.sub_state == "SEARCHING":
            self._pub_status("camera")

            if self.detected_obj == "orange_flare":
                self.get_logger().info("👁 Orange flare terdeteksi!")
                self.flare_position_memory = ""
                self._change_state("CENTERING")

            elif self._elapsed() > SEARCHING_TIMEOUT:
                self.get_logger().warn(
                    f"⏱ Timeout {SEARCHING_TIMEOUT}s — orange flare tidak ditemukan."
                )
                self._surface()

        # ── CENTERING ─────────────────────────────────────
        # Luruskan kamera ke flare.
        # Simpan posisi awal flare (kiri/kanan) dari x_diff.
        # Harus berada dalam toleransi selama CENTERING_HOLD_SEC detik berturut.
        elif self.sub_state == "CENTERING":
            self._pub_status("camera_yaw")

            # Simpan posisi flare pertama kali masuk CENTERING
            if self.flare_position_memory == "":
                self.flare_position_memory = "right" if self.x_diff > 0 else "left"
                self.get_logger().info(
                    f"📍 Posisi flare: {self.flare_position_memory} "
                    f"(x_diff={self.x_diff})"
                )

            if abs(self.x_diff) <= CAMERA_TOL:
                # Dalam toleransi → jalankan timer centering
                if self.within_tolerance_start is None:
                    self.within_tolerance_start = time.time()

                centered_for = time.time() - self.within_tolerance_start
                self.get_logger().info(
                    f"🎯 Dalam toleransi selama {centered_for:.1f}/{CENTERING_HOLD_SEC}s "
                    f"(x_diff={self.x_diff})"
                )

                if centered_for >= CENTERING_HOLD_SEC:
                    self._change_state("SWAYING")
            else:
                # Keluar toleransi → reset timer centering SAJA (bukan state timer)
                if self.within_tolerance_start is not None:
                    self.get_logger().warn(
                        f"⚠ Keluar toleransi (x_diff={self.x_diff}), reset timer centering"
                    )
                self.within_tolerance_start = None

        # ── SWAYING ───────────────────────────────────────
        # Sway berlawanan arah dari posisi flare selama SWAYING_SEC detik.
        # Flare di kanan → sway kiri (robot bergerak menjauhi flare ke kiri).
        # Flare di kiri  → sway kanan.
        elif self.sub_state == "SWAYING":
            sway_dir = "sway_left" if self.flare_position_memory == "right" else "sway_right"
            self._pub_status(sway_dir)
            self.get_logger().info(
                f"↔ Swaying {sway_dir} ({self._elapsed():.1f}/{SWAYING_SEC}s)"
            )

            if self._elapsed() >= SWAYING_SEC:
                self._change_state("FORWARD_PASS")

        # ── FORWARD_PASS ──────────────────────────────────
        # Maju penuh selama FORWARD_PASS_SEC detik untuk melewati area flare.
        elif self.sub_state == "FORWARD_PASS":
            self._pub_status("all")
            self.get_logger().info(
                f"⏩ Forward pass ({self._elapsed():.1f}/{FORWARD_PASS_SEC}s)"
            )

            if self._elapsed() >= FORWARD_PASS_SEC:
                self._surface()

        # ── DONE ──────────────────────────────────────────
        # Robot sudah surface, terus publish dpr_ssy.
        elif self.sub_state == "DONE":
            self._pub_status("dpr_ssy")


def main(args=None):
    rclpy.init(args=args)
    node = Flag1Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()