#!/usr/bin/env python3
"""
SAUVC Map Node (Dead Reckoning)
================================
Estimasi posisi robot berdasarkan kecepatan × waktu × arah (dead reckoning).
Publish /robot_pose dan Zone ke guidance.

Fix dari versi lama:
  1. Topic mismatch  : Subscribe 'status'/'boost'/'set_point' (lowercase),
                       sesuai dengan yang dipublish guidance baru.
  2. Relay dihapus   : Subscribe langsung ke /sensor_msg (Sensor) untuk dapat
                       yaw — node_old_pub_yaw_only.py tidak perlu dijalankan.
  3. Flag subscriber : Subscribe 'flag' (Int16) dari guidance agar koreksi
                       posisi robot_y bisa berjalan. Flag numbering diupdate:
                       flag 2 = ram flare (bukan 3 seperti sebelumnya).
  4. move_robot      : Tambah tracking posisi untuk sway_right, sway_left,
                       sway_right_forward, sway_left_forward.
                       Sway bergerak tegak lurus arah hadap robot (robot_theta ± π/2).
  5. speed_map       : Tambah entri untuk semua status yang dipakai guidance:
                       yaw_right, camera_yaw, camera_sway, dpr, dpr_ssy.
                       Status yang tidak menggerakkan robot (yaw, dpr) = 0,
                       eksplisit agar tidak ambigu dengan "key not found".
  6. Boost-aware sway: Sway sekarang lookup boost value yang diterima,
                       bukan hardcode boost=0.
"""

import threading
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Int8, Int16
from geometry_msgs.msg import Pose, Point, Quaternion
from auv_interfaces.msg import SetPoint, Sensor   # Sensor langsung, tanpa relay

# ── State global (dibaca dari dua thread: move_robot & publish_pose_loop) ──
robot_x     = 0.0
robot_y     = 0.0
robot_theta = np.radians(90)   # radian; awal hadap +Y (maju)
initial_yaw = 268.0 # masukin setpoint yaw depan
boost       = 0.0
status      = ""

receive_set_point       = False
receive_first_flag_2    = False   # FIX: dulu flag 3, sekarang flag 2 = ram flare

lock = threading.Lock()

# ── Plot ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 8))

# ============================================================
# SPEED MAP  ← tambah/edit sesuai hasil pengukuran di kolam
# ============================================================
#
# Kecepatan dalam meter per tick (tick = 0.1 detik = 10 Hz)
# Pengukuran asli:
#   all  boost=500 : 7.4 m / 10 s = 0.74 m/s → 0.074 m/tick
#   all  boost=350 : 6.2 m / 10 s = 0.62 m/s → 0.062 m/tick   (≈ 0.075 di kode lama, pakai yg lama)
#   camera boost=350: 4.6 m / 10 s            → 0.046 m/tick   (≈ 0.0408 di kode lama)
#   sway        : 2.4 m / 5 s = 0.48 m/s      → 0.048 m/tick
#
# FIX: tambah entri untuk semua status yang dipakai guidance,
#      agar calculate_speed tidak silent-return 0 dan menyebabkan map
#      berhenti tracking padahal robot masih bergerak.
speed_map = {
    # ── Maju ──────────────────────────────────────────────
    # ("all",           500): 0.085,
    # ("all",           350): 0.075,
    ("all",             0): 0.0625,

    # ── Mundur ────────────────────────────────────────────
    # ("backward",      350): 0.0788,
    ("backward",        0): 0.0625,

    # ── Camera (maju lambat karena kamera aktif) ──────────
    # ("camera",        350): 0.0408,
    ("camera",          0): 0.0625, #0.028,

    # ── Camera + yaw correction (kecepatan maju sedikit) ─
    # ("camera_yaw",    350): 0.020,
    ("camera_yaw",      0): 0.015,

    # ── Camera + sway correction ──────────────────────────
    # ("camera_sway",   350): 0.020,
    ("camera_sway",     0): 0.015,  

    # ── Last slow ─────────────────────────────────────────
    ("last_slow",     350): 0.0408,
    ("last_slow",       0): 0.032,

    # ── Sway (gerak lateral) ──────────────────────────────
    # ("sway_right",    350): 0.048,
    ("sway_right",      0): 0.0334,
    # ("sway_left",     350): 0.048,
    ("sway_left",       0): 0.0334,

    # ── Sway + maju bersamaan ─────────────────────────────
    ("sway_right_forward", 350): 0.048,
    ("sway_right_forward",   0): 0.048,
    ("sway_left_forward",  350): 0.048,
    ("sway_left_forward",    0): 0.048,

    # ── Yaw / putar ditempat → tidak geser posisi ─────────
    # FIX: eksplisit 0 agar tidak ambigu dengan "key not found"
    ("yaw_right",     350): 0.0,
    ("yaw_right",       0): 0.0,
    ("yaw_left",      350): 0.0,
    ("yaw_left",        0): 0.0,

    # ── DPR / surfacing → tidak geser posisi XY ───────────
    ("dpr",           350): 0.0,
    ("dpr",             0): 0.0,
    ("dpr_ssy",       350): 0.0,
    ("dpr_ssy",         0): 0.0,

    # ── Stop ──────────────────────────────────────────────
    ("stop",          350): 0.0,
    ("stop",            0): 0.0,
}


def calculate_speed(status_str: str, boost_val: float) -> float:
    """
    Cari kecepatan dari speed_map.
    Kalau tidak ketemu persis, coba fallback ke boost=350, lalu 0.
    """
    spd = speed_map.get((status_str, int(boost_val)))
    if spd is None:
        spd = speed_map.get((status_str, 350), 0.0)
    return spd


# ════════════════════════════════════════════════════════════
class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')

        # ── Publishers ──────────────────────────────────────
        self.pose_pub = self.create_publisher(Pose,  '/robot_pose', 10)
        self.zone_pub = self.create_publisher(Int8,  'Zone',        10)

        # ── Subscribers ─────────────────────────────────────
        # FIX 1: nama topic lowercase, sesuai guidance baru
        self.create_subscription(String,   'status',    self.status_callback,    10)
        self.create_subscription(Float32,  'boost',     self.boost_callback,     10)
        self.create_subscription(SetPoint, 'set_point', self.setPoint_callback,  10)

        # FIX 2: langsung dari sensor, tanpa relay node
        self.create_subscription(Sensor,   '/sensor_msg', self.sensor_callback,  10)

        # FIX 3: subscribe flag dari guidance
        self.create_subscription(Int16,    'flag',      self.flag_callback,      10)

    # ── Callbacks ───────────────────────────────────────────

    def boost_callback(self, msg: Float32):
        global boost
        boost = msg.data

    def setPoint_callback(self, msg: SetPoint):
        global initial_yaw, receive_set_point
        if not receive_set_point:
            initial_yaw = msg.yaw
            self.get_logger().info(f"Initial yaw set: {initial_yaw:.2f}")
            receive_set_point = True

    def sensor_callback(self, msg: Sensor):
        # FIX 2: ambil yaw langsung dari Sensor, tidak butuh relay node
        global robot_theta
        with lock:
            # yaw relatif terhadap arah awal
            relative_yaw = (msg.yaw - initial_yaw) % 360

            # OPTIONAL: kalau arah kebalik, aktifkan ini
            relative_yaw = -relative_yaw

            # convert ke radian (0° = hadap +X, 90 hadap +Y)
            robot_theta = np.radians(relative_yaw + 90)

    def status_callback(self, msg: String):
        global status
        status = msg.data.strip().lower()

    def flag_callback(self, msg: Int16):
        # FIX 3: update flag numbering
        # Flag 2 = awal ram flare (bukan 3 seperti versi lama)
        # Saat masuk flag 2 pertama kali, koreksi robot_y ke 14
        # (robot dianggap sudah melewati area awal dan ada di tengah kolam)
        global robot_y, receive_first_flag_2
        if msg.data == 2 and not receive_first_flag_2:
            with lock:
                robot_y = 5.0
            receive_first_flag_2 = True
            self.get_logger().info("Flag 2 diterima – robot_y dikoreksi ke 14.0")

    def publish_pose_loop(self):
        """Thread: publish pose dan zone ke guidance setiap 0.1 s."""
        while rclpy.ok():
            with lock:
                # Publish pose
                pose_msg = Pose()
                pose_msg.position    = Point(x=robot_x, y=robot_y, z=0.0)
                pose_msg.orientation = Quaternion(
                    x=0.0, y=0.0,
                    z=float(np.sin(robot_theta / 2)),
                    w=float(np.cos(robot_theta / 2))
                )
                self.pose_pub.publish(pose_msg)

                # Tentukan zona berdasarkan koordinat
                zone = self._get_zone(robot_x, robot_y)
                self.zone_pub.publish(Int8(data=zone))

            time.sleep(0.1)

    @staticmethod
    def _get_zone(x: float, y: float) -> int:
        """
        Bagi kolam menjadi 4 zona + zona luar (5):
          Zone 0 : area start / tengah bawah (y < 12)
          Zone 1 : kanan atas  (18≤y≤25,  0≤x≤14)
          Zone 2 : kiri atas   (18≤y≤25, -14≤x≤0)
          Zone 3 : kiri tengah (12≤y≤18, -14≤x≤0)
          Zone 4 : kanan tengah(12≤y≤18,  0≤x≤14)
          Zone 5 : out-of-bounds
        """
        if y >= 24 or x <= -4 or x >= 4:
            return 5
        if 10 <= y <= 12 and  0 <= x <= 5:
            return 1
        if 10 <= y <= 12 and -5 <= x <= 0:
            return 2
        if 7 <= y <= 9 and -5 <= x <= 0:
            return 3
        if 7 <= y <= 9 and  0 <= x <= 5:
            return 4
        return 0   # area start


# ════════════════════════════════════════════════════════════
def move_robot(node: RobotVisualizer):
    """
    Thread: update posisi robot berdasarkan status dan boost.

    FIX 4: tambah tracking sway (gerak lateral tegak lurus robot_theta).
    Sway ke kanan  = arah (robot_theta + π/2)
    Sway ke kiri   = arah (robot_theta - π/2)
    """
    global robot_x, robot_y

    while rclpy.ok():
        with lock:
            spd = calculate_speed(status, boost)

            if status == "stop":
                robot_x = 0.0
                robot_y = 0.0

            # ── Maju ──────────────────────────────────────
            elif status in ("all", "camera", "camera_yaw",
                            "camera_sway", "last_slow"):
                robot_x += spd * np.cos(robot_theta)
                robot_y += spd * np.sin(robot_theta)

            # ── Mundur ────────────────────────────────────
            elif status == "backward":
                robot_x -= spd * np.cos(robot_theta)
                robot_y -= spd * np.sin(robot_theta)

            # ── Sway kanan (lateral kanan) ─────────────────
            # FIX 4: dulu tidak dihandle → drift besar saat flag 1 sway
            elif status in ("sway_right", "sway_right_forward"):
                perp = robot_theta - np.pi / 2
                robot_x += spd * np.cos(perp)
                robot_y += spd * np.sin(perp)

            # ── Sway kiri (lateral kiri) ───────────────────
            elif status in ("sway_left", "sway_left_forward"):
                perp = robot_theta + np.pi / 2
                robot_x += spd * np.cos(perp)
                robot_y += spd * np.sin(perp)

            # ── Yaw / dpr → tidak geser posisi XY ──────────
            # (robot_theta diupdate dari yaw_callback, bukan di sini)
            # else: diam

            node.get_logger().info(
                f"[MAP] status={status!r:16s} boost={boost:.0f} "
                f"spd={spd:.4f} pos=({robot_x:.2f}, {robot_y:.2f})"
            )

        time.sleep(0.1)


# ════════════════════════════════════════════════════════════
trail_points: list = []


def setup_plot():
    ax.clear()
    ax.set_xlim(-5, 5)  #ukuran kolam
    ax.set_ylim(0, 25)  #
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Robot Dead-Reckoning Map")
    ax.grid(True, alpha=0.4)

    # Gambar batas zona
    zone_lines = [
        # (x1, y1, x2, y2, label, label_x, label_y)
        (0,  12, 0,  25, "",     0,    0),
        (-14, 18, 14, 18, "",    0,    0),
        (-14, 12, 14, 12, "",    0,    0),
    ]
    for x1, y1, x2, y2, lbl, lx, ly in zone_lines:
        ax.plot([x1, x2], [y1, y2], 'k--', linewidth=0.5, alpha=0.4)

    # # Label zona  ukuran untuk sauvc
    # for label, cx, cy in [
    #     ("Z1", 7, 14), ("Z2", -7, 14),
    #     ("Z3", -7, 10),  ("Z4",  7, 10),
    #     ("Z0", 0, 5),
    # ]:
    #     ax.text(cx, cy, label, ha='center', va='center',
    #             fontsize=9, color='gray', alpha=0.6)
        
    # Label zona untuk tes di kolam manapun
    for label, cx, cy in [
        ("Z1", 3, 14), ("Z2", -3, 14),
        ("Z3", -3, 10),  ("Z4",  3, 10),
        ("Z0", 0, 5),
    ]:
        ax.text(cx, cy, label, ha='center', va='center',
                fontsize=9, color='gray', alpha=0.6)


def update_plot(frame):
    setup_plot()
    with lock:
        trail_points.append((robot_x, robot_y))

    # Plot trail
    if len(trail_points) > 1:
        xs, ys = zip(*trail_points)
        ax.plot(xs, ys, 'b-', linewidth=1.2, alpha=0.7)

    # Plot posisi saat ini
    with lock:
        cx, cy = robot_x, robot_y
    ax.plot(cx, cy, 'ro', markersize=8)
    ax.annotate(f"({cx:.1f}, {cy:.1f})", (cx, cy),
                textcoords="offset points", xytext=(6, 6), fontsize=8)


# ════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = RobotVisualizer()

    threading.Thread(target=node.publish_pose_loop,          daemon=True).start()
    threading.Thread(target=move_robot, args=(node,),        daemon=True).start()
    threading.Thread(target=lambda: rclpy.spin(node),        daemon=True).start()

    ani = animation.FuncAnimation(fig, update_plot, interval=100)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()