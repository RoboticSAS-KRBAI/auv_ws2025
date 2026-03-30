#!/usr/bin/env python3
import rclpy
from auv_interfaces.msg import ObjectDetection, ObjectDifference, SetPoint, MultiPID, PID
from std_msgs.msg import String, Float32

from rclpy.node import Node
import time
import math

# ============================================================
# STATE MACHINE STATES
# ============================================================
# PHASE 1: Maju scanning
STATE_FORWARD_SCAN = "FORWARD_SCAN"

# PHASE 1 -> Kemungkinan 1 & 2: Orange Flare terdeteksi
STATE_FLARE_AIM = "FLARE_AIM"               # dpr_ssy, aim ke flare
STATE_FLARE_HOLD = "FLARE_HOLD"             # Hold 3 detik stabil
STATE_FLARE_SWAY_AVOID = "FLARE_SWAY_AVOID" # Sway menghindari flare
STATE_FLARE_FORWARD_TO_GATE = "FLARE_FORWARD_TO_GATE"  # Maju cari gate setelah menghindari

# PHASE 2: Cari gate
STATE_SEARCH_GATE_FORWARD = "SEARCH_GATE_FORWARD"  # Maju fy2 cari gate 5 detik

# PHASE 2 -> Kemungkinan 1: Gate terdeteksi
STATE_GATE_AIM_SWAY = "GATE_AIM_SWAY"       # dpr_ssy, sway ke tengah gate
STATE_GATE_HOLD = "GATE_HOLD"                # Hold 3 detik stabil
STATE_GATE_ENTER = "GATE_ENTER"              # Maju masuk gate (status all)
STATE_GATE_LOST_COUNT = "GATE_LOST_COUNT"    # Gate hilang, hitung 2 detik sebelum stop

# PHASE 2 -> Kemungkinan 2: Gate tidak terdeteksi, putar tau
STATE_GATE_SPIN_SEARCH = "GATE_SPIN_SEARCH"  # Putar CW/CCW cari gate

# PHASE 2 -> Kemungkinan 2 lanjut: Gate terdeteksi saat spin
STATE_GATE_SPIN_FOUND_SWAY = "GATE_SPIN_FOUND_SWAY"  # Sway cepat ke arah gate
STATE_GATE_SPIN_STABILIZE = "GATE_SPIN_STABILIZE"     # Sway pelan stabilize

# PHASE 2 -> Kemungkinan 3: Spin gagal, sway cari
STATE_GATE_SWAY_SEARCH = "GATE_SWAY_SEARCH"  # Sway pelan cari gate

STATE_MISSION_COMPLETE = "MISSION_COMPLETE"


class MissionAccumulator(Node):
    def __init__(self):
        super().__init__('accumulator_subscriber')

        # ========== PUBLISHERS ==========
        self.pub_object_difference = self.create_publisher(ObjectDifference, 'object_difference', 10)
        self.pub_status = self.create_publisher(String, 'status', 10)
        self.pub_set_point = self.create_publisher(SetPoint, 'set_point', 10)
        self.pub_multi_pid = self.create_publisher(MultiPID, 'pid', 10)
        self.pub_boost = self.create_publisher(Float32, 'boost', 10)

        # ========== SUBSCRIBER ==========
        self.sub_object_detection = self.create_subscription(
            ObjectDetection,
            'object_detection',
            self.object_detection_callback,
            10
        )

        # ========== FRAME CONFIG ==========
        self.frame_center_x = 640 // 2  # 320
        self.stable_threshold = 30       # Pixel threshold untuk dianggap "stabil di tengah"

        # ========== DETECTION DATA (updated by callback) ==========
        self.flare_detected = False
        self.flare_x_diff = 0            # Pixel diff flare dari center
        self.flare_bbox_size = 0

        self.gate_detected = False
        self.gate_x_diff = 0             # Pixel diff gate dari center
        self.gate_bbox_size = 0

        # ========== STATE MACHINE ==========
        self.state = STATE_FORWARD_SCAN
        self.state_start_time = 0.0
        self.is_started = False

        # ========== MISSION MEMORY ==========
        self.flare_was_on_left = False    # True jika flare ada di kiri saat aim
        self.avoid_sway_direction = ""    # "sway_right" atau "sway_left"
        self.setpoint_yaw = 270.0         # Setpoint yaw misi (sesuaikan)

        # Hold stability tracking
        self.hold_start_time = 0.0
        self.hold_stable_duration = 0.0
        self.hold_required = 3.0          # 3 detik harus stabil

        # Gate spin tracking
        self.spin_start_yaw = 0.0
        self.spin_total_degrees = 0.0
        self.spin_max_degrees = 720.0     # Max 2 putaran 360
        self.spin_direction = ""          # "yaw_right" (CW) atau "yaw_left" (CCW)
        self.spin_found_gate = False
        self.spin_found_yaw = 0.0         # Yaw saat gate pertama terdeteksi di spin
        self.spin_found_gate_side = ""    # "left" atau "right" relatif terhadap setpoint

        # Gate enter tracking
        self.gate_lost_time = 0.0         # Waktu terakhir gate hilang

        # ========== SET POINT ==========
        self.set_point = SetPoint()
        self.set_point.yaw = self.setpoint_yaw
        self.set_point.pitch = 0.0
        self.set_point.roll = 0.0
        self.set_point.depth = -0.4

        # ========== PID CONFIG ==========
        self.multi_pid = MultiPID()
        pid_yaw = PID(); pid_yaw.kp = 5.0; pid_yaw.ki = 0.0; pid_yaw.kd = 0.3
        pid_pitch = PID(); pid_pitch.kp = 15.0; pid_pitch.ki = 0.0; pid_pitch.kd = 2.6
        pid_roll = PID(); pid_roll.kp = 700.0; pid_roll.ki = 0.0; pid_roll.kd = 0.0
        pid_depth = PID(); pid_depth.kp = 3000.0; pid_depth.ki = 0.0; pid_depth.kd = 0.0
        pid_camera = PID(); pid_camera.kp = 1.0; pid_camera.ki = 0.0; pid_camera.kd = 0.0
        self.multi_pid.pid_yaw = pid_yaw
        self.multi_pid.pid_pitch = pid_pitch
        self.multi_pid.pid_roll = pid_roll
        self.multi_pid.pid_depth = pid_depth
        self.multi_pid.pid_camera = pid_camera

        # Initial publish
        self.publish_status("stop")
        self.pub_set_point.publish(self.set_point)
        self.pub_multi_pid.publish(self.multi_pid)
        self.publish_boost(0.0)

        # ========== TIMER (10 Hz state machine loop) ==========
        self.timer = self.create_timer(0.1, self.mission_loop)

        self.get_logger().info("=== MISSION ACCUMULATOR INITIALIZED ===")
        self.get_logger().info(f"Setpoint Yaw: {self.setpoint_yaw}")

    # ============================================================
    # HELPER: PUBLISHERS
    # ============================================================
    def publish_status(self, status_str):
        msg = String()
        msg.data = status_str
        self.pub_status.publish(msg)

    def publish_boost(self, boost_val):
        msg = Float32()
        msg.data = boost_val
        self.pub_boost.publish(msg)

    def publish_object_difference(self, obj_type, x_diff, is_target=False, bbox_size=0):
        msg = ObjectDifference()
        msg.object_type = obj_type
        msg.x_difference = int(x_diff)
        msg.is_target = is_target
        msg.bounding_box_size = int(bbox_size)
        self.pub_object_difference.publish(msg)

    def get_elapsed(self):
        """Waktu sejak state saat ini dimulai"""
        return time.time() - self.state_start_time

    def switch_state(self, new_state):
        self.get_logger().info(f"STATE: {self.state} -> {new_state}")
        self.state = new_state
        self.state_start_time = time.time()
        # Reset hold tracking saat pindah state
        self.hold_start_time = 0.0
        self.hold_stable_duration = 0.0

    def is_centered(self, x_diff):
        """Cek apakah objek sudah di tengah kamera (dalam threshold)"""
        return abs(x_diff) <= self.stable_threshold

    # ============================================================
    # OBJECT DETECTION CALLBACK
    # ============================================================
    def object_detection_callback(self, data):
        self.flare_detected = False
        self.gate_detected = False

        for bbox in data.bounding_boxes:
            center_x = (bbox.x_min + bbox.x_max) // 2
            x_difference = center_x - self.frame_center_x
            bbox_width = bbox.x_max - bbox.x_min

            if bbox.class_name == "orange_flare":
                self.flare_detected = True
                self.flare_x_diff = x_difference
                self.flare_bbox_size = bbox_width

            elif bbox.class_name == "gate":
                self.gate_detected = True
                self.gate_x_diff = x_difference
                self.gate_bbox_size = bbox_width

        # Publish object difference (prioritas flare saat phase 1, gate saat phase 2)
        if self.state in [STATE_FORWARD_SCAN, STATE_FLARE_AIM, STATE_FLARE_HOLD,
                          STATE_FLARE_SWAY_AVOID, STATE_FLARE_FORWARD_TO_GATE]:
            if self.flare_detected:
                self.publish_object_difference("orange_flare", self.flare_x_diff,
                                                True, self.flare_bbox_size)
            else:
                self.publish_object_difference("None", 0)
        else:
            if self.gate_detected:
                self.publish_object_difference("gate", self.gate_x_diff,
                                                True, self.gate_bbox_size)
            else:
                self.publish_object_difference("None", 0)

    # ============================================================
    # HOLD STABILITY CHECKER
    # ============================================================
    def check_hold_stability(self, x_diff, required_seconds=3.0):
        """
        Return True jika objek sudah stabil di tengah selama required_seconds.
        Jika tidak stabil, reset timer hold.
        """
        if self.is_centered(x_diff):
            if self.hold_start_time == 0.0:
                self.hold_start_time = time.time()
            self.hold_stable_duration = time.time() - self.hold_start_time
            if self.hold_stable_duration >= required_seconds:
                return True
        else:
            # Gagal stabil -> reset hold timer
            self.hold_start_time = 0.0
            self.hold_stable_duration = 0.0
        return False

    # ============================================================
    # MAIN MISSION LOOP (10 Hz)
    # ============================================================
    def mission_loop(self):
        if not self.is_started:
            self.is_started = True
            self.state_start_time = time.time()
            self.get_logger().info("=== MISSION STARTED ===")

        # Selalu publish setpoint dan PID
        self.pub_set_point.publish(self.set_point)
        self.pub_multi_pid.publish(self.multi_pid)

        # ==================================================================
        # PHASE 1: MAJU SCANNING (4 detik, status "all")
        # ==================================================================
        if self.state == STATE_FORWARD_SCAN:
            self.publish_status("all") #all
            elapsed = self.get_elapsed()
            self.get_logger().info(f"[FORWARD_SCAN] Maju scanning... {elapsed:.1f}/4.0s | "
                                    f"Flare:{self.flare_detected} Gate:{self.gate_detected}")

            # Kemungkinan 1 & 2: Terdeteksi orange flare
            if self.flare_detected:
                self.get_logger().info("[FORWARD_SCAN] Flare terdeteksi! Mulai aim ke flare.")
                self.flare_was_on_left = (self.flare_x_diff < 0)
                self.switch_state(STATE_FLARE_AIM)
                return

            # Kemungkinan 3: 4 detik habis, tidak ada flare -> langsung cari gate
            if elapsed >= 4.0:
                self.get_logger().info("[FORWARD_SCAN] 4s habis, tidak ada flare. Cari gate.")
                self.switch_state(STATE_SEARCH_GATE_FORWARD)
                return

        # ==================================================================
        # FLARE AIM: dpr_ssy, aim titik tengah kamera ke flare
        # ==================================================================
        elif self.state == STATE_FLARE_AIM:
            self.publish_status("dpr_ssy")
            self.get_logger().info(f"[FLARE_AIM] Aiming ke flare... x_diff={self.flare_x_diff}")

            if not self.flare_detected:
                # Flare hilang saat aim -> anggap tidak ada halangan, cari gate
                self.get_logger().info("[FLARE_AIM] Flare hilang. Skip ke cari gate.")
                self.switch_state(STATE_SEARCH_GATE_FORWARD)
                return

            # Update sisi flare
            self.flare_was_on_left = (self.flare_x_diff < 0)

            # Cek apakah sudah di tengah -> mulai hold
            if self.is_centered(self.flare_x_diff):
                self.get_logger().info("[FLARE_AIM] Flare di tengah! Mulai hold 3 detik.")
                self.switch_state(STATE_FLARE_HOLD)
                return

            # Belum di tengah -> publish camera error agar teensy adjust via camera_yaw
            # Gunakan status "camera_yaw" agar teensy putar ke arah flare
            self.publish_object_difference("orange_flare", self.flare_x_diff, True, self.flare_bbox_size)

        # ==================================================================
        # FLARE HOLD: Stabilize 3 detik menghadap flare
        # ==================================================================
        elif self.state == STATE_FLARE_HOLD:
            self.publish_status("dpr_ssy")

            if not self.flare_detected:
                # Flare hilang -> cari gate
                self.get_logger().info("[FLARE_HOLD] Flare hilang saat hold. Cari gate.")
                self.switch_state(STATE_SEARCH_GATE_FORWARD)
                return

            stable = self.check_hold_stability(self.flare_x_diff, self.hold_required)
            self.get_logger().info(f"[FLARE_HOLD] Holding... stable_dur={self.hold_stable_duration:.1f}/{self.hold_required}s "
                                    f"x_diff={self.flare_x_diff} centered={self.is_centered(self.flare_x_diff)}")

            if not self.is_centered(self.flare_x_diff):
                # Keluar dari tengah -> kembali aim
                self.get_logger().info("[FLARE_HOLD] Gagal stabil, kembali aim.")
                self.switch_state(STATE_FLARE_AIM)
                return

            if stable:
                # Berhasil hold 3 detik! Tentukan arah sway menghindari
                if self.flare_was_on_left:
                    # Flare awalnya di kiri -> sway kanan untuk menghindari
                    self.avoid_sway_direction = "sway_right"
                else:
                    # Flare awalnya di kanan -> sway kiri untuk menghindari
                    self.avoid_sway_direction = "sway_left"
                self.get_logger().info(f"[FLARE_HOLD] Stabil 3s! Avoid dengan {self.avoid_sway_direction}")
                self.switch_state(STATE_FLARE_SWAY_AVOID)
                return

        # ==================================================================
        # FLARE SWAY AVOID: Sway menghindari flare selama 2 detik
        # ==================================================================
        elif self.state == STATE_FLARE_SWAY_AVOID:
            self.publish_status(self.avoid_sway_direction)
            elapsed = self.get_elapsed()
            self.get_logger().info(f"[FLARE_SWAY_AVOID] {self.avoid_sway_direction} {elapsed:.1f}/2.0s")

            if elapsed >= 2.0:
                self.get_logger().info("[FLARE_SWAY_AVOID] 2s selesai. Maju cari gate.")
                self.switch_state(STATE_FLARE_FORWARD_TO_GATE)
                return

        # ==================================================================
        # FLARE FORWARD TO GATE: Maju "all" sambil cari gate 5 detik
        # ==================================================================
        elif self.state == STATE_FLARE_FORWARD_TO_GATE:
            self.publish_status("all") #all
            elapsed = self.get_elapsed()
            self.get_logger().info(f"[FLARE_FWD_GATE] Maju cari gate {elapsed:.1f}/5.0s | Gate:{self.gate_detected}")

            if self.gate_detected:
                self.get_logger().info("[FLARE_FWD_GATE] Gate terdeteksi! Mulai aim gate.")
                self.switch_state(STATE_GATE_AIM_SWAY)
                return

            if elapsed >= 5.0:
                self.get_logger().info("[FLARE_FWD_GATE] 5s habis. Pindah ke search gate forward.")
                self.switch_state(STATE_SEARCH_GATE_FORWARD)
                return

        # ==================================================================
        # PHASE 2: SEARCH GATE FORWARD (maju fy2, 5 detik)
        # ==================================================================
        elif self.state == STATE_SEARCH_GATE_FORWARD:
            # Gunakan "all" sebagai pengganti fy2 (forward with yaw PID)
            # Jika teensy punya "fy2" bisa ganti di sini
            self.publish_status("all") #all
            elapsed = self.get_elapsed()
            self.get_logger().info(f"[SEARCH_GATE_FWD] Maju cari gate {elapsed:.1f}/5.0s | Gate:{self.gate_detected}")

            # Kemungkinan 1: Gate terdeteksi
            if self.gate_detected:
                self.get_logger().info("[SEARCH_GATE_FWD] Gate terdeteksi! Aim dengan sway.")
                self.switch_state(STATE_GATE_AIM_SWAY)
                return

            # Kemungkinan 2: 5 detik habis, gate tidak terdeteksi -> putar cari
            if elapsed >= 5.0:
                self.get_logger().info("[SEARCH_GATE_FWD] 5s habis, gate tidak terdeteksi. Mulai spin search.")
                # Tentukan arah spin berdasarkan arah avoid sebelumnya
                if self.avoid_sway_direction == "sway_right":
                    self.spin_direction = "yaw_left"   # CCW
                elif self.avoid_sway_direction == "sway_left":
                    self.spin_direction = "yaw_right"  # CW
                else:
                    self.spin_direction = "yaw_right"  # Default CW
                self.spin_total_degrees = 0.0
                self.spin_found_gate = False
                self.switch_state(STATE_GATE_SPIN_SEARCH)
                return

        # ==================================================================
        # GATE AIM SWAY: dpr_ssy, sway ke arah gate sampai tengah
        # ==================================================================
        elif self.state == STATE_GATE_AIM_SWAY:
            self.set_point.yaw = self.setpoint_yaw
            self.pub_set_point.publish(self.set_point)

            if not self.gate_detected:
                # Gate hilang saat aim -> kembali search
                self.get_logger().info("[GATE_AIM_SWAY] Gate hilang, kembali search.")
                self.switch_state(STATE_SEARCH_GATE_FORWARD)
                return

            self.get_logger().info(f"[GATE_AIM_SWAY] Sway ke gate... x_diff={self.gate_x_diff}")

            if self.gate_x_diff > self.stable_threshold:
                # Gate di kanan -> sway kanan
                self.publish_status("sway_right")
            elif self.gate_x_diff < -self.stable_threshold:
                # Gate di kiri -> sway kiri
                self.publish_status("sway_left")
            else:
                # Sudah di tengah -> dpr_ssy dan mulai hold
                self.publish_status("dpr_ssy")
                self.get_logger().info("[GATE_AIM_SWAY] Gate di tengah! Mulai hold.")
                self.switch_state(STATE_GATE_HOLD)
                return

        # ==================================================================
        # GATE HOLD: Stabilize 3 detik (gate di tengah pakai sway)
        # ==================================================================
        elif self.state == STATE_GATE_HOLD:
            self.set_point.yaw = self.setpoint_yaw
            self.pub_set_point.publish(self.set_point)

            if not self.gate_detected:
                # Gate hilang saat hold -> kembali aim
                self.get_logger().info("[GATE_HOLD] Gate hilang, kembali aim.")
                self.switch_state(STATE_GATE_AIM_SWAY)
                return

            # Masih perlu sway untuk maintain posisi
            if self.gate_x_diff > self.stable_threshold:
                self.publish_status("sway_right")
                # Reset hold karena belum stabil
                self.hold_start_time = 0.0
                self.hold_stable_duration = 0.0
                self.get_logger().info(f"[GATE_HOLD] Koreksi sway_right x_diff={self.gate_x_diff}")
            elif self.gate_x_diff < -self.stable_threshold:
                self.publish_status("sway_left")
                self.hold_start_time = 0.0
                self.hold_stable_duration = 0.0
                self.get_logger().info(f"[GATE_HOLD] Koreksi sway_left x_diff={self.gate_x_diff}")
            else:
                self.publish_status("dpr_ssy")
                stable = self.check_hold_stability(self.gate_x_diff, self.hold_required)
                self.get_logger().info(f"[GATE_HOLD] Holding... {self.hold_stable_duration:.1f}/{self.hold_required}s "
                                        f"x_diff={self.gate_x_diff}")

                if stable:
                    self.get_logger().info("[GATE_HOLD] Stabil 3s! MAJU MASUK GATE!")
                    self.switch_state(STATE_GATE_ENTER)
                    return

        # ==================================================================
        # GATE ENTER: Maju masuk gate (status all)
        # ==================================================================
        elif self.state == STATE_GATE_ENTER:
            self.publish_status("all") #all
            self.get_logger().info(f"[GATE_ENTER] Maju masuk gate! Gate detected: {self.gate_detected}")

            if not self.gate_detected:
                # Gate hilang -> mulai hitung 2 detik
                if self.gate_lost_time == 0.0:
                    self.gate_lost_time = time.time()
                    self.get_logger().info("[GATE_ENTER] Gate hilang, mulai hitung 2 detik...")

                lost_duration = time.time() - self.gate_lost_time
                self.get_logger().info(f"[GATE_ENTER] Gate lost {lost_duration:.1f}/2.0s")

                if lost_duration >= 2.0:
                    self.get_logger().info("[GATE_ENTER] 2s tanpa gate. MISSION COMPLETE!")
                    self.switch_state(STATE_MISSION_COMPLETE)
                    return
            else:
                # Gate masih terlihat -> reset lost timer
                self.gate_lost_time = 0.0

        # ==================================================================
        # GATE SPIN SEARCH: Putar CW/CCW cari gate (max 720 derajat)
        # ==================================================================
        elif self.state == STATE_GATE_SPIN_SEARCH:
            self.publish_status("dpr_ssy")  # Stabilize dulu sebentar

            # Putar perlahan
            self.publish_status(self.spin_direction)
            elapsed = self.get_elapsed()

            # Estimasi derajat: yaw_right/left di teensy = ±0.3 tau
            # Kira-kira ~15-20 deg/sec tergantung thruster, pakai estimasi waktu
            # 360 derajat ~ 18 detik (20 deg/s), 720 derajat ~ 36 detik
            estimated_deg_per_sec = 20.0
            self.spin_total_degrees = elapsed * estimated_deg_per_sec

            self.get_logger().info(f"[GATE_SPIN] {self.spin_direction} | est_deg={self.spin_total_degrees:.0f}/720 "
                                    f"| Gate:{self.gate_detected}")

            # Gate terdeteksi saat spin!
            if self.gate_detected:
                self.spin_found_gate = True
                self.spin_found_yaw = self.setpoint_yaw  # Simpan (simplified)
                # Tentukan sisi gate relatif terhadap setpoint yaw
                if self.gate_x_diff < 0:
                    self.spin_found_gate_side = "left"
                else:
                    self.spin_found_gate_side = "right"

                self.get_logger().info(f"[GATE_SPIN] Gate ditemukan! Side: {self.spin_found_gate_side}")

                # Set setpoint yaw ke setpoint misi
                self.set_point.yaw = self.setpoint_yaw
                self.pub_set_point.publish(self.set_point)

                self.switch_state(STATE_GATE_SPIN_FOUND_SWAY)
                return

            # Kemungkinan 3: Spin selesai 720 derajat tanpa menemukan gate
            if self.spin_total_degrees >= self.spin_max_degrees:
                self.get_logger().info("[GATE_SPIN] 720 derajat, gate tidak ditemukan. Sway search.")
                self.switch_state(STATE_GATE_SWAY_SEARCH)
                return

        # ==================================================================
        # GATE SPIN FOUND SWAY: Gate ditemukan saat spin, sway cepat ke gate
        # ==================================================================
        elif self.state == STATE_GATE_SPIN_FOUND_SWAY:
            self.set_point.yaw = self.setpoint_yaw
            self.pub_set_point.publish(self.set_point)

            if not self.gate_detected:
                # Gate hilang lagi saat sway cepat -> kembali search
                elapsed = self.get_elapsed()
                if elapsed > 3.0:
                    self.get_logger().info("[SPIN_FOUND_SWAY] Gate hilang >3s, kembali sway search.")
                    self.switch_state(STATE_GATE_SWAY_SEARCH)
                    return
                # Tetap sway ke arah terakhir
                if self.spin_found_gate_side == "left":
                    self.publish_status("sway_left")
                else:
                    self.publish_status("sway_right")
                self.get_logger().info(f"[SPIN_FOUND_SWAY] Gate hilang, tetap sway {self.spin_found_gate_side}")
                return

            self.get_logger().info(f"[SPIN_FOUND_SWAY] Sway cepat ke gate x_diff={self.gate_x_diff}")

            # Sway cepat ke arah gate
            if self.spin_found_gate_side == "left":
                self.publish_status("sway_left")
            else:
                self.publish_status("sway_right")

            # Sudah cukup dekat ke tengah -> pindah ke stabilize pelan
            if abs(self.gate_x_diff) < self.stable_threshold * 2:
                self.get_logger().info("[SPIN_FOUND_SWAY] Mendekati tengah, mulai stabilize.")
                self.switch_state(STATE_GATE_SPIN_STABILIZE)
                return

        # ==================================================================
        # GATE SPIN STABILIZE: Sway pelan stabilize + hold 3 detik
        # ==================================================================
        elif self.state == STATE_GATE_SPIN_STABILIZE:
            self.set_point.yaw = self.setpoint_yaw
            self.pub_set_point.publish(self.set_point)

            if not self.gate_detected:
                self.get_logger().info("[SPIN_STABILIZE] Gate hilang, kembali sway search.")
                self.switch_state(STATE_GATE_SWAY_SEARCH)
                return

            # Sway koreksi pelan
            if self.gate_x_diff > self.stable_threshold:
                self.publish_status("sway_right")
                self.hold_start_time = 0.0
                self.hold_stable_duration = 0.0
            elif self.gate_x_diff < -self.stable_threshold:
                self.publish_status("sway_left")
                self.hold_start_time = 0.0
                self.hold_stable_duration = 0.0
            else:
                self.publish_status("dpr_ssy")
                stable = self.check_hold_stability(self.gate_x_diff, self.hold_required)
                self.get_logger().info(f"[SPIN_STABILIZE] Hold {self.hold_stable_duration:.1f}/{self.hold_required}s "
                                        f"x_diff={self.gate_x_diff}")

                if stable:
                    self.get_logger().info("[SPIN_STABILIZE] Stabil 3s! MAJU MASUK GATE!")
                    self.switch_state(STATE_GATE_ENTER)
                    return

        # ==================================================================
        # GATE SWAY SEARCH: Sway pelan cari gate (kemungkinan 3)
        # ==================================================================
        elif self.state == STATE_GATE_SWAY_SEARCH:
            self.set_point.yaw = self.setpoint_yaw
            self.pub_set_point.publish(self.set_point)

            # Arah sway kebalikan dari avoid sebelumnya
            if self.avoid_sway_direction == "sway_left":
                search_sway = "sway_right"
            elif self.avoid_sway_direction == "sway_right":
                search_sway = "sway_left"
            else:
                search_sway = "sway_right"  # default

            self.publish_status(search_sway)
            self.get_logger().info(f"[SWAY_SEARCH] {search_sway} cari gate... Gate:{self.gate_detected}")

            if self.gate_detected:
                self.get_logger().info("[SWAY_SEARCH] Gate ditemukan! Mulai aim sway.")
                self.switch_state(STATE_GATE_AIM_SWAY)
                return

        # ==================================================================
        # MISSION COMPLETE: STOP
        # ==================================================================
        elif self.state == STATE_MISSION_COMPLETE:
            self.publish_status("stop")
            self.get_logger().info("[MISSION COMPLETE] AUV stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = MissionAccumulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()