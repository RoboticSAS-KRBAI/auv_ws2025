#!/usr/bin/env python3
"""
TEST FLAG 3 — Pass Through Gate
==================================
Jalankan file ini untuk test Flag 3 saja secara isolated.
Robot dimulai dari posisi default (tengah map), maju mencari gate,
lalu melewatinya dan surfacing.

Alur:
  INIT → TO_DEFAULT → FORWARD_BLIND → APPROACH_GATE → DONE (surface)
                              ↓ (timeout 8s, gate tidak terdeteksi)
                         SEARCH_GATE → APPROACH_GATE → DONE

Cara pakai:
  ros2 run auv_pkg node_test_flag3
  (atau: python3 node_test_flag3.py)

Yang perlu jalan bersamaan:
  - node_object_detection
  - node_sauvc_accumulator_fadhil
  - node_map_sauvc   ← WAJIB (untuk /robot_pose)
  - node kontrol motor (teensy)
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDifference

# ══════════════════════════════════════════════════════════════
# KONFIGURASI  ← edit sesuai kondisi lapangan
# ══════════════════════════════════════════════════════════════

# Posisi default sebelum maju ke gate
DEFAULT_POS = (0.0, 6.0)   # (x, y) meter

POSITION_TOL       = 1.0   # meter, toleransi navigasi ke posisi default
FORWARD_BLIND_SEC  = 8.0   # detik maju buta sebelum beralih ke SEARCH_GATE
GATE_LOST_SEC      = 2.0   # detik gate tidak terdeteksi → dianggap terlewati
# ══════════════════════════════════════════════════════════════


class Flag3Node(Node):
    def __init__(self):
        super().__init__('test_flag3')

        self.sub_state              = "INIT"
        self.state_start_time       = time.time()
        self.within_tolerance_start = None   # timer gate-hilang independen

        self.x_diff       = 0
        self.detected_obj = "None"

        self.robot_x = 0.0
        self.robot_y = 0.0

        # ── Publishers ──────────────────────────────────────
        self.pub_multi_pid     = self.create_publisher(MultiPID, "pid",           10)
        self.pub_set_point     = self.create_publisher(SetPoint, "set_point",     10)
        self.pub_status        = self.create_publisher(String,   "status",        10)
        self.pub_active_target = self.create_publisher(String,   "active_target", 10)
        self.pub_boost         = self.create_publisher(Float32,  "boost",         10)

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(
            ObjectDifference, 'object_difference', self.obj_diff_cb, 10)
        self.create_subscription(
            Pose, '/robot_pose', self.pose_cb, 10)

        # ── SetPoint & PID awal ─────────────────────────────
        self.set_point       = SetPoint()
        self.set_point.yaw   = 0.0
        self.set_point.pitch = 0.0
        self.set_point.roll  = 1.2
        self.set_point.depth = -0.11

        self.multi_pid_msg            = MultiPID()
        pid_yaw                       = PID(); pid_yaw.kp = 4.5;    pid_yaw.ki = 0.0;    pid_yaw.kd = 0.3
        pid_pitch                     = PID(); pid_pitch.kp = 15.0; pid_pitch.ki = 0.0;  pid_pitch.kd = 2.6
        pid_roll                      = PID(); pid_roll.kp = 2.5;   pid_roll.ki = 0.0;   pid_roll.kd = 0.4
        pid_depth                     = PID(); pid_depth.kp = 1350.0; pid_depth.ki = 0.0; pid_depth.kd = 215.0
        pid_camera                    = PID(); pid_camera.kp = 0.5; pid_camera.ki = 0.0; pid_camera.kd = 0.0
        self.multi_pid_msg.pid_yaw    = pid_yaw
        self.multi_pid_msg.pid_pitch  = pid_pitch
        self.multi_pid_msg.pid_roll   = pid_roll
        self.multi_pid_msg.pid_depth  = pid_depth
        self.multi_pid_msg.pid_camera = pid_camera

        self.pub_multi_pid.publish(self.multi_pid_msg)
        self.pub_set_point.publish(self.set_point)
        self._pub_boost(350.0)

        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info(
            f"🟢 [FLAG 3 TEST] Node started\n"
            f"   Posisi default : {DEFAULT_POS}\n"
            f"   Forward blind  : {FORWARD_BLIND_SEC}s\n"
            f"   Gate lost thr  : {GATE_LOST_SEC}s"
        )

    # ── Callbacks ───────────────────────────────────────────
    def obj_diff_cb(self, msg: ObjectDifference):
        self.detected_obj = msg.object_type
        self.x_diff       = msg.x_difference

    def pose_cb(self, msg: Pose):
        self.robot_x = msg.position.x
        self.robot_y = msg.position.y

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

    def _at_default(self) -> bool:
        dx = DEFAULT_POS[0]
        dy = DEFAULT_POS[1]
        return (abs(self.robot_x - dx) < POSITION_TOL and
                abs(self.robot_y - dy) < POSITION_TOL)

    def _navigate_to_default(self):
        """Gerak sederhana menuju posisi default."""
        dx = DEFAULT_POS[0]
        dy = DEFAULT_POS[1]
        dy_diff = dy - self.robot_y
        dx_diff = dx - self.robot_x
        if abs(dy_diff) > POSITION_TOL:
            self._pub_status("all" if dy_diff > 0 else "backward")
        elif abs(dx_diff) > POSITION_TOL:
            self._pub_status("sway_right" if dx_diff > 0 else "sway_left")

    def _surface(self):
        self.get_logger().info("✅ [FLAG 3] Gate terlewati! Surfacing...")
        self.set_point.depth = 0.0
        self.pub_set_point.publish(self.set_point)
        self._change_state("DONE")

    # ── Main loop ───────────────────────────────────────────
    def loop(self):
        self.pub_multi_pid.publish(self.multi_pid_msg)
        self.pub_set_point.publish(self.set_point)
        self.pub_active_target.publish(String(data="gate"))

        # ── INIT ──────────────────────────────────────────
        if self.sub_state == "INIT":
            self.get_logger().info("🚪 [FLAG 3] INIT → TO_DEFAULT")
            self._change_state("TO_DEFAULT")

        # ── TO_DEFAULT ────────────────────────────────────
        # Kembali ke posisi tengah map terlebih dahulu.
        # Penting agar robot mulai dari titik yang konsisten,
        # terutama jika test Flag 3 dijalankan setelah Flag 2 selesai
        # dan robot berada di zona yang berbeda.
        elif self.sub_state == "TO_DEFAULT":
            if self._at_default():
                self.get_logger().info(
                    f"📍 Sudah di posisi default "
                    f"({self.robot_x:.2f}, {self.robot_y:.2f})"
                )
                self._change_state("FORWARD_BLIND")
            else:
                self.get_logger().info(
                    f"🔁 Menuju default {DEFAULT_POS} — "
                    f"posisi saat ini ({self.robot_x:.2f}, {self.robot_y:.2f})"
                )
                self._navigate_to_default()

                # Jika gate sudah terdeteksi selama navigasi, langsung approach
                if self.detected_obj == "gate":
                    self.get_logger().info(
                        "👁 Gate terdeteksi saat TO_DEFAULT → langsung APPROACH"
                    )
                    self._change_state("APPROACH_GATE")

        # ── FORWARD_BLIND ─────────────────────────────────
        # Maju lurus dengan status "all" selama FORWARD_BLIND_SEC detik.
        # Jika gate terdeteksi sebelum timeout → langsung APPROACH_GATE.
        # Jika timeout tercapai tanpa deteksi → SEARCH_GATE.
        elif self.sub_state == "FORWARD_BLIND":
            self._pub_status("all")
            self.get_logger().info(
                f"⏩ Forward blind ({self._elapsed():.1f}/{FORWARD_BLIND_SEC}s) — "
                f"terdeteksi: {self.detected_obj}"
            )

            if self.detected_obj == "gate":
                self.get_logger().info("👁 Gate terdeteksi! → APPROACH_GATE")
                self._change_state("APPROACH_GATE")

            elif self._elapsed() > FORWARD_BLIND_SEC:
                self.get_logger().warn(
                    f"⏱ {FORWARD_BLIND_SEC}s berlalu, gate belum terdeteksi → SEARCH_GATE"
                )
                self._change_state("SEARCH_GATE")

        # ── SEARCH_GATE ───────────────────────────────────
        # Berhenti maju, gunakan kamera (status "camera") untuk mencari gate.
        # Tidak ada timeout di sini — robot akan terus mencari sampai ketemu.
        # Jika perlu timeout, tambahkan kondisi self._elapsed() > X.
        elif self.sub_state == "SEARCH_GATE":
            self._pub_status("camera")
            self.get_logger().info(
                f"🔍 Mencari gate... ({self._elapsed():.1f}s) — "
                f"terdeteksi: {self.detected_obj}"
            )

            if self.detected_obj == "gate":
                self.get_logger().info("👁 Gate ditemukan! → APPROACH_GATE")
                self._change_state("APPROACH_GATE")

        # ── APPROACH_GATE ─────────────────────────────────
        # Ikuti gate dengan kamera sambil maju.
        # Gate dianggap "terlewati" jika tidak terdeteksi selama GATE_LOST_SEC detik.
        #
        # Catatan: within_tolerance_start dipakai sebagai timer "gate hilang",
        # bukan timer "dalam toleransi centering".
        # Logika:
        #   - gate terlihat  → reset timer gate-hilang, terus maju
        #   - gate hilang    → mulai timer gate-hilang
        #   - timer ≥ GATE_LOST_SEC → gate sudah terlewati → surface
        elif self.sub_state == "APPROACH_GATE":
            self._pub_status("camera")

            if self.detected_obj == "gate":
                # Gate masih terlihat → reset timer gate-hilang
                if self.within_tolerance_start is not None:
                    self.get_logger().info("Gate terlihat kembali, reset timer gate-hilang")
                self.within_tolerance_start = None
                self.get_logger().info(
                    f"🚪 Mendekati gate (x_diff={self.x_diff})"
                )
            else:
                # Gate tidak terdeteksi → mulai/lanjutkan timer
                if self.within_tolerance_start is None:
                    self.within_tolerance_start = time.time()
                    self.get_logger().warn("⚠ Gate hilang dari kamera, mulai timer...")

                lost_for = time.time() - self.within_tolerance_start
                self.get_logger().warn(
                    f"⚠ Gate hilang {lost_for:.1f}/{GATE_LOST_SEC}s"
                )

                if lost_for >= GATE_LOST_SEC:
                    self._surface()

        # ── DONE ──────────────────────────────────────────
        elif self.sub_state == "DONE":
            self._pub_status("dpr_ssy")


def main(args=None):
    rclpy.init(args=args)
    node = Flag3Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()