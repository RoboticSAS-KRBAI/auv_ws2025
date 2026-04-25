#!/usr/bin/env python3
"""
TEST FLAG 2 — Ram 3 Coloured Flares
======================================
Jalankan file ini untuk test Flag 2 saja secara isolated.
Robot dimulai dari posisi default (tengah map), lalu menabrak
3 flare sesuai urutan dan zona yang dikonfigurasi.

Alur per flare:
  INIT → TO_DEFAULT → TO_ZONE → SCANNING → CENTERING_FLARE → RAMMING
       → BACKING_UP → (ulang untuk flare berikutnya)
  Setelah semua flare selesai → DONE (surface)

Cara pakai:
  ros2 run auv_pkg node_test_flag2
  (atau: python3 node_test_flag2.py)

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

# Urutan flare yang ditabrak
FLARE_ORDER = ["red_flare", "blue_flare", "yellow_flare"]

# Zona tempat setiap flare berada (1-4)
FLARE_ZONE = {
    "red_flare":    1,
    "blue_flare":   3,
    "yellow_flare": 3,
}

# Koordinat tengah tiap zona (meter, sesuai sistem koordinat map)
ZONE_CENTER = {
    0: ( 0.0,  6.0),   # posisi default / tengah dekat start
    1: ( 7.0, 21.0),
    2: (-7.0, 21.0),
    3: (-7.0, 15.0),
    4: ( 7.0, 15.0),
}

POSITION_TOL       = 1.0    # meter, toleransi posisi robot
CAMERA_TOL         = 30     # pixel, toleransi centering kamera
BBOX_NEAR          = 300    # pixel lebar bbox dianggap "sudah dekat"
SCANNING_TIMEOUT   = 20.0   # detik, timeout scanning sebelum skip flare
CENTERING_HOLD_SEC = 1.5    # detik, harus di tengah sebelum ramming
RAMMING_TIMEOUT    = 5.0    # detik, fallback timeout ramming
BACKING_SEC        = 2.0    # detik, durasi mundur setelah tabrak
# ══════════════════════════════════════════════════════════════


class Flag2Node(Node):
    def __init__(self):
        super().__init__('test_flag2')

        self.sub_state              = "INIT"
        self.state_start_time       = time.time()
        self.within_tolerance_start = None

        self.x_diff       = 0
        self.bbox_size    = 0
        self.detected_obj = "None"

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.current_flare_idx = 0   # indeks ke FLARE_ORDER saat ini

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
            f"🟢 [FLAG 2 TEST] Node started\n"
            f"   Urutan flare : {FLARE_ORDER}\n"
            f"   Zona flare   : {FLARE_ZONE}"
        )

    # ── Callbacks ───────────────────────────────────────────
    def obj_diff_cb(self, msg: ObjectDifference):
        self.detected_obj = msg.object_type
        self.x_diff       = msg.x_difference
        self.bbox_size    = getattr(msg, 'bounding_box_size', 0)

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

    def _at_position(self, tx: float, ty: float) -> bool:
        return (abs(self.robot_x - tx) < POSITION_TOL and
                abs(self.robot_y - ty) < POSITION_TOL)

    def _navigate_to(self, tx: float, ty: float):
        """Gerak sederhana berbasis selisih koordinat. Prioritas Y dulu."""
        dy = ty - self.robot_y
        dx = tx - self.robot_x
        if abs(dy) > POSITION_TOL:
            self._pub_status("all" if dy > 0 else "backward")
        elif abs(dx) > POSITION_TOL:
            self._pub_status("sway_right" if dx > 0 else "sway_left")

    def _surface(self):
        self.get_logger().info("✅ [FLAG 2] Semua flare selesai! Surfacing...")
        self.set_point.depth = 0.0
        self.pub_set_point.publish(self.set_point)
        self._change_state("DONE")

    # ── Main loop ───────────────────────────────────────────
    def loop(self):
        self.pub_multi_pid.publish(self.multi_pid_msg)
        self.pub_set_point.publish(self.set_point)

        # Semua flare selesai
        if self.current_flare_idx >= len(FLARE_ORDER):
            self._surface()
            return

        target_flare = FLARE_ORDER[self.current_flare_idx]
        target_zone  = FLARE_ZONE[target_flare]
        default_x, default_y = ZONE_CENTER[0]
        zone_x,    zone_y    = ZONE_CENTER[target_zone]

        self.pub_active_target.publish(String(data=target_flare))

        # ── INIT ──────────────────────────────────────────
        if self.sub_state == "INIT":
            self.get_logger().info(
                f"🎯 Target [{self.current_flare_idx+1}/{len(FLARE_ORDER)}]: "
                f"{target_flare} → Zona {target_zone} "
                f"({zone_x}, {zone_y})"
            )
            self._change_state("TO_DEFAULT")

        # ── TO_DEFAULT ────────────────────────────────────
        # Kembali ke posisi tengah/default dulu.
        # Memastikan robot berangkat dari titik yang sama ke setiap zona.
        elif self.sub_state == "TO_DEFAULT":
            if self._at_position(default_x, default_y):
                self.get_logger().info(
                    f"📍 Sudah di posisi default "
                    f"({self.robot_x:.2f}, {self.robot_y:.2f})"
                )
                self._change_state("TO_ZONE")
            else:
                self.get_logger().info(
                    f"🔁 Menuju default ({default_x}, {default_y}) — "
                    f"posisi saat ini ({self.robot_x:.2f}, {self.robot_y:.2f})"
                )
                self._navigate_to(default_x, default_y)

        # ── TO_ZONE ───────────────────────────────────────
        # Pergi ke tengah zona target.
        elif self.sub_state == "TO_ZONE":
            if self._at_position(zone_x, zone_y):
                self.get_logger().info(
                    f"📍 Sudah di zona {target_zone} "
                    f"({self.robot_x:.2f}, {self.robot_y:.2f})"
                )
                self._change_state("SCANNING")
            else:
                self.get_logger().info(
                    f"🔁 Menuju zona {target_zone} ({zone_x}, {zone_y}) — "
                    f"posisi saat ini ({self.robot_x:.2f}, {self.robot_y:.2f})"
                )
                self._navigate_to(zone_x, zone_y)

        # ── SCANNING ──────────────────────────────────────
        # Putar di tempat sambil cari flare target.
        # Timeout SCANNING_TIMEOUT detik → skip flare ini.
        elif self.sub_state == "SCANNING":
            self._pub_status("yaw_right")
            self.get_logger().info(
                f"🔄 Scanning {target_flare} "
                f"({self._elapsed():.1f}/{SCANNING_TIMEOUT}s) — "
                f"terdeteksi: {self.detected_obj}"
            )

            if self.detected_obj == target_flare:
                self.get_logger().info(f"👁 {target_flare} terdeteksi!")
                self._change_state("CENTERING_FLARE")

            elif self._elapsed() > SCANNING_TIMEOUT:
                self.get_logger().warn(
                    f"⏱ Timeout scanning {target_flare}, skip ke flare berikutnya."
                )
                self.current_flare_idx += 1
                self._change_state("INIT")

        # ── CENTERING_FLARE ───────────────────────────────
        # Luruskan kamera ke flare sebelum menabrak.
        # Jika flare hilang dari kamera, kembali ke SCANNING.
        elif self.sub_state == "CENTERING_FLARE":
            self._pub_status("camera_yaw")

            if self.detected_obj != target_flare:
                self.get_logger().warn(
                    f"⚠ {target_flare} hilang saat centering → kembali SCANNING"
                )
                self._change_state("SCANNING")
                return

            if abs(self.x_diff) <= CAMERA_TOL:
                if self.within_tolerance_start is None:
                    self.within_tolerance_start = time.time()
                hold = time.time() - self.within_tolerance_start
                self.get_logger().info(
                    f"🎯 Centering: {hold:.1f}/{CENTERING_HOLD_SEC}s "
                    f"(x_diff={self.x_diff})"
                )
                if hold >= CENTERING_HOLD_SEC:
                    self._change_state("RAMMING")
            else:
                self.within_tolerance_start = None
                self.get_logger().info(
                    f"↔ Meluruskan kamera ke {target_flare} "
                    f"(x_diff={self.x_diff})"
                )

        # ── RAMMING ───────────────────────────────────────
        # Maju menabrak flare. Berhenti jika bbox sudah besar
        # (BBOX_NEAR pixel) atau timeout RAMMING_TIMEOUT detik.
        elif self.sub_state == "RAMMING":
            self._pub_status("camera")
            self.get_logger().info(
                f"💥 Ramming {target_flare} — "
                f"bbox={self.bbox_size}px, "
                f"elapsed={self._elapsed():.1f}/{RAMMING_TIMEOUT}s"
            )

            if self.bbox_size >= BBOX_NEAR:
                self.get_logger().info(
                    f"✅ Bbox cukup besar ({self.bbox_size}px) → BACKING_UP"
                )
                self._change_state("BACKING_UP")
            elif self._elapsed() > RAMMING_TIMEOUT:
                self.get_logger().warn(
                    f"⏱ Ramming timeout → BACKING_UP"
                )
                self._change_state("BACKING_UP")

        # ── BACKING_UP ────────────────────────────────────
        # Mundur BACKING_SEC detik setelah tabrak, lalu ke flare berikutnya.
        elif self.sub_state == "BACKING_UP":
            self._pub_status("backward")
            self.get_logger().info(
                f"⬅ Mundur ({self._elapsed():.1f}/{BACKING_SEC}s)"
            )

            if self._elapsed() > BACKING_SEC:
                self.current_flare_idx += 1
                self.get_logger().info(
                    f"✅ {target_flare} selesai! "
                    f"({self.current_flare_idx}/{len(FLARE_ORDER)} flare)"
                )
                self._change_state("INIT")

        # ── DONE ──────────────────────────────────────────
        elif self.sub_state == "DONE":
            self._pub_status("dpr_ssy")


def main(args=None):
    rclpy.init(args=args)
    node = Flag2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()