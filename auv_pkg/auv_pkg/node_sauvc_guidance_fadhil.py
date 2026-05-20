#!/usr/bin/env python3
"""
SAUVC Guidance Node – State Machine WITH MAP
====================================
Flag 1 : Dodge orange flare
Flag 2 : Ram 3 coloured flares (urutan & zona bisa dikonfigurasi)
Flag 3 : Pass through gate
Flag 4 : Surface (drop bucket / mission done)

Perbaikan dari versi sebelumnya:
  1. Bug centering Flag-1  : timer "sudah di tengah" pakai variabel tersendiri
                             (within_tolerance_start), bukan state_start_time.
                             Sebelumnya me-reset state_start_time sehingga
                             kondisi elapsed > 3 s tidak pernah tercapai.
  2. Timeout Flag-1        : setelah timeout, setpoint dipublish ulang sebelum
                             pindah ke Flag 2 (kembalikan yaw ke setpoint awal).
  3. Flag-2 navigasi       : ganti logika "if zone != 0 → backward" yang terlalu
                             simplistis dengan navigasi berbasis koordinat robot
                             dari node map (/robot_pose).
  4. Flag-2 SCANNING       : tambahkan timeout agar tidak scan selamanya.
  5. Flag-2 CENTERING_FLARE: state baru antara SCANNING → RAMMING; robot luruskan
                             kamera sebelum menabrak, agar tidak nyasar.
  6. Flag-2 RAMMING        : pakai bbox_size (dari accumulator) sebagai pendeteksi
                             "sudah dekat" selain fallback timeout.
  7. Flag-3                : tambah state TO_DEFAULT sebelum FORWARD_BLIND; robot
                             kembali ke posisi tengah map dulu agar sinkron.
  8. Flag-3 gate hilang    : ganti self.state_start_time = time.time() (salah)
                             dengan within_tolerance_start yang independen.
  9. Flag-4                : tambah state SURFACING agar setpoint depth = 0 hanya
                             dipublish sekali, bukan setiap 0.1 s tanpa henti.
 10. Tambah pub_boost      : publisher Boost yang hilang di versi sebelumnya.
 11. Tambah pose_cb        : subscribe /robot_pose untuk baca koordinat robot.
 12. Accumulator           : publish bounding_box_size → dipakai di RAMMING.
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int8
from geometry_msgs.msg import Pose
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDifference

# ============================================================
# KONFIGURASI MISI  ← ubah di sini sesuai kondisi lapangan
# ============================================================

# Urutan flare yang ditabrak
FLARE_ORDER = ["blue_flare", "red_flare", "yellow_flare"]

# Zona tempat setiap flare berada (1-4)
FLARE_ZONE = {
    "red_flare":    2,
    "blue_flare":   3,
    "yellow_flare": 4,
}

# Koordinat tengah setiap zona & posisi default (zona 0 = tengah/start)
# Sesuaikan dengan ukuran kolam dan posisi starting box
ZONE_CENTER = { #sesuaiin environment arena
    0: ( 0.0,  5.0),   # default / tengah map dekat starting
    1: ( 4.0, 14.0),
    2: (-4.0, 14.0),
    3: (-4.0, 10.0),
    4: ( 4.0, 10.0),
}

# Toleransi posisi robot (meter)
POSITION_TOL = 1.0

# Toleransi kamera (pixel) untuk centering
CAMERA_TOL = 30

# Ukuran bbox (pixel lebar) yang dianggap "sudah dekat" saat ramming
BBOX_NEAR = 300

# ============================================================


class SubGuidance(Node):
    def __init__(self):
        super().__init__('guidance_teensy')

        # ── FSM ─────────────────────────────────────────────
        self.mission_flag = 1          # flag aktif saat ini
        self.sub_state    = "INIT"
        self.state_start_time = time.time()

        # Sensor / deteksi
        self.x_diff       = 0
        self.bbox_size    = 0          # dari accumulator
        self.detected_obj = "None"

        # Flag-1 memory
        self.flare_position_memory  = ""   # "left" | "right"
        self.within_tolerance_start = None # timer centering independen

        # Flag-2 state
        self.current_flare_idx = 0

        # Posisi robot (dari node map)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # ── Publishers ──────────────────────────────────────
        self.pub_multi_pid     = self.create_publisher(MultiPID, "pid",           10)
        self.pub_set_point     = self.create_publisher(SetPoint, "set_point",     10)
        self.pub_status        = self.create_publisher(String,   "status",        10)
        self.pub_active_target = self.create_publisher(String,   "active_target", 10)
        self.pub_boost         = self.create_publisher(Float32,  "boost",         10)

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(ObjectDifference, 'object_difference',
                                 self.obj_diff_cb, 10)
        self.create_subscription(Int8,  'Zone',        self.zone_cb,  10)
        self.create_subscription(Pose,  '/robot_pose', self.pose_cb,  10)

        # ── SetPoint awal ───────────────────────────────────
        self.set_point = SetPoint()
        self.set_point.yaw   = 268.0
        self.set_point.pitch = 0.0
        self.set_point.roll  = 1.4
        self.set_point.depth = -0.34

        # ── PID ─────────────────────────────────────────────
        self.multi_pid_msg = MultiPID()

        pid_yaw           = PID()
        pid_yaw.kp        = 4.5
        pid_yaw.ki        = 0.0
        pid_yaw.kd        = 0.3

        pid_pitch         = PID()
        pid_pitch.kp      = 9.0
        pid_pitch.ki      = 0.0
        pid_pitch.kd      = 1.7

        pid_roll          = PID()
        pid_roll.kp       = 2.5
        pid_roll.ki       = 0.0
        pid_roll.kd       = 0.3

        pid_depth         = PID()
        pid_depth.kp      = 1350.0
        pid_depth.ki      = 0.0
        pid_depth.kd      = 215.0

        pid_camera        = PID()
        pid_camera.kp     = 0.11
        pid_camera.ki     = 0.0
        pid_camera.kd     = 0.0

        self.multi_pid_msg.pid_yaw    = pid_yaw
        self.multi_pid_msg.pid_pitch  = pid_pitch
        self.multi_pid_msg.pid_roll   = pid_roll
        self.multi_pid_msg.pid_depth  = pid_depth
        self.multi_pid_msg.pid_camera = pid_camera

        self.pub_multi_pid.publish(self.multi_pid_msg)
        self.pub_set_point.publish(self.set_point)
        self._publish_boost(0.0)

        # ── Main loop 10 Hz ─────────────────────────────────
        self.timer = self.create_timer(0.1, self.mission_loop)
        self.get_logger().info("✅ Guidance State Machine Started!")

    # ════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════

    def obj_diff_cb(self, msg: ObjectDifference):
        self.detected_obj = msg.object_type
        self.x_diff       = msg.x_difference
        # bounding_box_size ada di accumulator versi baru
        self.bbox_size    = getattr(msg, 'bounding_box_size', 0)

    def zone_cb(self, msg):
        # Zone dari node map (Int8)
        pass  # tidak dipakai langsung; navigasi pakai koordinat robot

    def pose_cb(self, msg: Pose):
        self.robot_x = msg.position.x
        self.robot_y = msg.position.y

    # ════════════════════════════════════════════════════════
    # HELPERS
    # ════════════════════════════════════════════════════════

    def publish_status(self, status_str: str):
        msg = String()
        msg.data = status_str
        self.pub_status.publish(msg)

    def _publish_boost(self, value: float):
        msg = Float32()
        msg.data = value
        self.pub_boost.publish(msg)

    def change_sub_state(self, new_state: str):
        """Ganti sub-state dan reset semua timer state."""
        self.sub_state              = new_state
        self.state_start_time       = time.time()
        self.within_tolerance_start = None        # selalu reset timer centering
        self.get_logger().info(
            f"[FLAG {self.mission_flag}] ──► {new_state}"
        )

    def elapsed(self) -> float:
        """Waktu (detik) sejak masuk sub_state saat ini."""
        return time.time() - self.state_start_time

    def at_position(self, tx: float, ty: float) -> bool:
        """True jika robot sudah berada dalam toleransi dari titik target."""
        return (abs(self.robot_x - tx) < POSITION_TOL and
                abs(self.robot_y - ty) < POSITION_TOL)

    def navigate_to(self, tx: float, ty: float):
        """
        Perintah gerak sederhana berdasarkan selisih koordinat.
        Prioritaskan Y dulu (maju/mundur), lalu X (sway).
        Sesuaikan dengan arah sumbu pada map yang dipakai.
        """
        dy = ty - self.robot_y
        dx = tx - self.robot_x

        forward_cmd = "all"
        backward_cmd = "backward"

        if abs(dy) > POSITION_TOL:
            if dy < 0:
                self.publish_status(forward_cmd)
            else:
                self.publish_status(backward_cmd)
        elif abs(dx) > POSITION_TOL:
            self.publish_status("sway_right" if dx > 0 else "sway_left")

    # ════════════════════════════════════════════════════════
    # MAIN LOOP
    # ════════════════════════════════════════════════════════

    def mission_loop(self):
        # Publish PID & setpoint setiap tick agar kontrol tidak hilang
        self.pub_multi_pid.publish(self.multi_pid_msg)
        self.pub_set_point.publish(self.set_point)

        # ──────────────────────────────────────────────────
        # FLAG 1 : DODGE ORANGE FLARE
        # ──────────────────────────────────────────────────
        if self.mission_flag == 1:
            self.pub_active_target.publish(String(data="orange_flare"))

            # ── INIT ──
            if self.sub_state == "INIT":
                self.change_sub_state("SEARCHING")

            # ── SEARCHING ──
            # Maju perlahan (camera) max 8 detik mencari orange flare
            elif self.sub_state == "SEARCHING":
                self.publish_status("all")

                if self.detected_obj == "orange_flare" and self.elapsed()>2.0:
                    self.publish_status("camera_yaw")
                    self.flare_position_memory = ""   # reset sebelum centering
                    self.change_sub_state("CENTERING")

                elif self.elapsed() > 10.0:
                    # Timeout → tidak ketemu; kembalikan setpoint lalu lanjut
                    self.get_logger().warn("⏱ Timeout Flag 1 – lanjut ke Flag 2")
                    self.pub_set_point.publish(self.set_point)   # kembalikan yaw setpoint
                    self.mission_flag = 2
                    self.change_sub_state("INIT")

            # ── CENTERING ──
            # Luruskan kamera ke flare (camera_yaw).
            # Simpan posisi flare (kiri/kanan).
            # Tunggu 3 detik benar-benar di tengah sebelum sway.
            #
            # BUG LAMA: menggunakan state_start_time sebagai timer "dalam toleransi"
            #           → timer di-reset juga saat change_sub_state
            #           → kondisi elapsed > 3 s tidak pernah terpenuhi jika robot
            #             sempat keluar toleransi sebentar.
            # FIX     : gunakan within_tolerance_start yang independen.
            elif self.sub_state == "CENTERING":
                self.publish_status("camera_yaw")

                # Simpan posisi flare pertama kali masuk CENTERING
                if self.flare_position_memory == "":
                    self.flare_position_memory = "right" if self.x_diff > 0 else "left"
                    self.get_logger().info(
                        f"Flare terdeteksi di: {self.flare_position_memory}"
                    )

                if abs(self.x_diff) <= CAMERA_TOL:
                    # Mulai timer "dalam toleransi"
                    if self.within_tolerance_start is None:
                        self.within_tolerance_start = time.time()
                    centered_for = time.time() - self.within_tolerance_start
                    if centered_for >= 1.0:
                        self.change_sub_state("SWAYING")
                else:
                    # Keluar toleransi → reset timer centering (bukan state_start!)
                    self.within_tolerance_start = None

            # ── SWAYING ──
            # Sway berlawanan arah flare selama 2 detik
            elif self.sub_state == "SWAYING":
                if self.flare_position_memory == "right":
                    self.publish_status("sway_left")
                else:
                    self.publish_status("sway_right")

                if self.elapsed() >= 2.0:
                    self.change_sub_state("FORWARD_PASS")

            # ── FORWARD_PASS ──
            # Maju penuh 3 detik untuk melewati area flare
            elif self.sub_state == "FORWARD_PASS":
                self.publish_status("all")
                if self.elapsed() >= 1.0:
                    self.get_logger().info("✅ Flag 1 Selesai!")
                    self.mission_flag = 2
                    self.change_sub_state("INIT")

        # ──────────────────────────────────────────────────
        # FLAG 2 : RAM 3 FLARES
        # ──────────────────────────────────────────────────
        elif self.mission_flag == 2:

            # Semua flare selesai → lanjut ke gate
            if self.current_flare_idx >= len(FLARE_ORDER):
                self.get_logger().info("✅ Semua flare ditabrak! Lanjut Flag 3.")
                self.mission_flag = 3
                self.change_sub_state("INIT")
                return

            target_flare = FLARE_ORDER[self.current_flare_idx]
            target_zone  = FLARE_ZONE[target_flare]
            self.pub_active_target.publish(String(data=target_flare))

            default_x, default_y = ZONE_CENTER[0]
            zone_x,    zone_y    = ZONE_CENTER[target_zone]

            # ── INIT ──
            # Log target dan langsung ke posisi default
            if self.sub_state == "INIT":
                self.get_logger().info(
                    f"Target [{self.current_flare_idx+1}/{len(FLARE_ORDER)}]: "
                    f"{target_flare} → Zona {target_zone}"
                )
                self.change_sub_state("TO_DEFAULT")

            # ── TO_DEFAULT ──
            # Kembali ke posisi tengah map sebelum pindah ke zona baru.
            # Ini memastikan robot selalu dari titik yang sama saat mulai
            # mencari setiap flare baru (lebih mudah dikontrol).
            #
            # BUG LAMA: hanya cek current_zone == 0 lalu "backward" buta;
            #           jika zone map tidak update cepat, robot tidak bergerak.
            # FIX     : navigasi berbasis koordinat aktual dari /robot_pose.
            elif self.sub_state == "TO_DEFAULT":

                if self.at_position(default_x, default_y):

                    # STOP dulu
                    self.publish_status("dpr_ssy")

                    # mulai timer kalau belum mulai
                    if self.within_tolerance_start is None:
                        self.within_tolerance_start = time.time()

                    # cek sudah diam 3 detik
                    if time.time() - self.within_tolerance_start >= 3.0:
                        self.change_sub_state("TO_ZONE")

                else:
                    # belum sampai → reset timer
                    self.within_tolerance_start = None
                    self.navigate_to(default_x, default_y)

            # ── TO_ZONE ──
            # Setelah di posisi default, pergi ke tengah zona target.
            #
            # BUG LAMA: cek current_zone == target_zone lalu "all" buta;
            #           jika di zona salah arah, robot tidak mengoreksi.
            # FIX     : navigate_to dengan koordinat.
            elif self.sub_state == "TO_ZONE":
                if self.at_position(zone_x, zone_y):
                    self.change_sub_state("SCANNING")
                else:
                    self.navigate_to(zone_x, zone_y)

            # ── SCANNING ──
            # Putar ditempat sambil cari flare target.
            # Timeout 20 s → skip flare ini (hindari stuck selamanya).
            elif self.sub_state == "SCANNING":
                self.publish_status("yaw_right")

                if self.detected_obj == target_flare:
                    self.change_sub_state("CENTERING_FLARE")

                elif self.elapsed() > 20.0:
                    self.get_logger().warn(
                        f"⏱ Timeout scanning {target_flare}, skip ke flare berikutnya."
                    )
                    self.current_flare_idx += 1
                    self.change_sub_state("INIT")

            # ── CENTERING_FLARE ──
            # Luruskan kamera ke flare sebelum menabrak.
            # State ini tidak ada di versi lama → robot langsung RAMMING
            # tanpa alignment → sering meleset.
            elif self.sub_state == "CENTERING_FLARE":
                self.publish_status("camera_yaw")

                # Jika flare hilang dari kamera → kembali scan
                if self.detected_obj != target_flare:
                    self.get_logger().warn("Flare hilang saat centering, kembali SCANNING.")
                    self.change_sub_state("SCANNING")
                    return

                if abs(self.x_diff) <= CAMERA_TOL:
                    if self.within_tolerance_start is None:
                        self.within_tolerance_start = time.time()
                    if time.time() - self.within_tolerance_start >= 1.5:
                        self.change_sub_state("RAMMING")
                else:
                    self.within_tolerance_start = None

            # ── RAMMING ──
            # Maju menabrak flare (status "camera" agar tetap lurus).
            # Hentikan jika bbox cukup besar (sudah sangat dekat) atau timeout.
            elif self.sub_state == "RAMMING":
                self.publish_status("camera")

                if self.bbox_size >= BBOX_NEAR or self.elapsed() > 5.0:
                    self.change_sub_state("BACKING_UP")

            # ── BACKING_UP ──
            # Mundur 2 detik lalu lanjut ke flare berikutnya
            elif self.sub_state == "BACKING_UP":
                self.publish_status("backward")
                if self.elapsed() > 2.0:
                    self.current_flare_idx += 1
                    self.get_logger().info(
                        f"✅ {target_flare} ditabrak! "
                        f"({self.current_flare_idx}/{len(FLARE_ORDER)})"
                    )
                    self.change_sub_state("INIT")

        # ──────────────────────────────────────────────────
        # FLAG 3 : GATE
        # ──────────────────────────────────────────────────
        elif self.mission_flag == 3:
            self.pub_active_target.publish(String(data="gate"))
            default_x, default_y = ZONE_CENTER[0]

            # ── INIT ──
            if self.sub_state == "INIT":
                self.change_sub_state("TO_DEFAULT")

            # ── TO_DEFAULT ──
            # Kembali ke posisi tengah sebelum maju mencari gate.
            # Tidak ada di versi lama → robot langsung maju dari posisi
            # mana pun setelah selesai Flag 2.
            elif self.sub_state == "TO_DEFAULT":
                if self.at_position(default_x, default_y):
                    self.change_sub_state("FORWARD_BLIND")
                else:
                    self.navigate_to(default_x, default_y)

            # ── FORWARD_BLIND ──
            # Maju 8 detik; jika gate terdeteksi langsung APPROACH
            elif self.sub_state == "FORWARD_BLIND":
                self.publish_status("all")
                if self.detected_obj == "gate":
                    self.change_sub_state("APPROACH_GATE")
                elif self.elapsed() > 8.0:
                    self.change_sub_state("SEARCH_GATE")

            # ── SEARCH_GATE ──
            # Berhenti dan cari gate dengan kamera (tidak maju)
            elif self.sub_state == "SEARCH_GATE":
                self.publish_status("camera")
                if self.detected_obj == "gate":
                    self.change_sub_state("APPROACH_GATE")

            # ── APPROACH_GATE ──
            # Ikuti gate dengan kamera. Jika gate hilang 2 detik → terlewati.
            #
            # BUG LAMA: self.state_start_time = time.time() ketika gate
            #           masih kelihatan → timer elapsed berulang dan kondisi
            #           elapsed > 2.0 tidak pernah tercapai saat gate hilang.
            #           Selain itu state ini pakai "camera_yaw" padahal
            #           maju ke gate cukup "camera".
            # FIX     : pakai within_tolerance_start sebagai timer gate hilang.
            elif self.sub_state == "APPROACH_GATE":
                self.publish_status("camera")

                if self.detected_obj != "gate":
                    # Gate tidak terdeteksi → mulai timer
                    if self.within_tolerance_start is None:
                        self.within_tolerance_start = time.time()
                    if time.time() - self.within_tolerance_start >= 2.0:
                        self.get_logger().info("✅ Gate terlewati! Lanjut Flag 4.")
                        self.mission_flag = 4
                        self.change_sub_state("INIT")
                else:
                    # Gate masih terlihat → reset timer gate-hilang
                    self.within_tolerance_start = None

        # ──────────────────────────────────────────────────
        # FLAG 4 : SURFACE (mission done)
        # ──────────────────────────────────────────────────
        elif self.mission_flag == 4:

            # ── INIT ──
            # Set depth ke 0 sekali, lalu pindah ke SURFACING.
            # BUG LAMA: set_point.depth = 0 dan publish_status dipanggil
            #           setiap 0.1 s tanpa state yang jelas.
            if self.sub_state == "INIT":
                self.get_logger().info("🏁 Mission Done – Surfacing!")
                self.set_point.depth = 0.0
                self.pub_set_point.publish(self.set_point)
                self.change_sub_state("SURFACING")

            # ── SURFACING ──
            elif self.sub_state == "SURFACING":
                self.publish_status("dpr_ssy")


# ════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = SubGuidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
