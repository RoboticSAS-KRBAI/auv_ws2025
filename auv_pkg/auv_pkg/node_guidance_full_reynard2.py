#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDifference, Sensor


COLOR_FLARES = {"red_flare", "yellow_flare", "blue_flare"}


class Guidance(Node):

    def __init__(self):
        super().__init__('guidance')

        # ── TIME ──────────────────────────────────────────────────────────
        self.start_time       = None
        self.state_start_time = None

        # ── DELAY ──────────────────────────────────────────────────────────
        self.param_delay = 5.0  # ← ganti nilai ini sesuai kebutuhan (detik)

        # ── STATE ─────────────────────────────────────────────────────────
        self.state = "INIT"

        # ── OBJECT INFO (dari ObjectDifference) ───────────────────────────
        self.object_class = ""
        self.is_target    = False

        # ── ORANGE FLARE SCAN ─────────────────────────────────────────────
        self.scan_left      = True
        self.last_scan_time = None
        self.orange_search_start = None

        # ── ORANGE FLARE TRACKING ─────────────────────────────────────────
        self.flare_lock_start           = None
        self.flare_orange_is_target_once   = False  # ← untuk SEARCH: sudah pernah is_target
        self.flare_orange_detected_once    = False  # ← untuk DODGE: sudah pernah detect
        self.last_orange_flare_seen     = 0

        # ── COLOR FLARE CHALLENGE ─────────────────────────────────────────
        self.color_flares_done    = set()
        self.current_color_flare  = None
        self.color_flare_hit_time = None        # kapan is_target pertama True
        self.color_flare_state    = "SEARCH"    # sub-state: SEARCH / APPROACH / BACK
        self._substate_start      = None
        self.color_flare_locked   = False
        self._is_target_since     = None

        # ── GATE TRACKING ─────────────────────────────────────────────────
        self.gate_detected_once = False
        self.last_gate_seen     = 0

        self.gate_sway_state      = None   # "ALIGN" / "SWAY" / "SCAN"
        self.gate_scan_start_time = None
        self.gate_sway_direction  = None   # "sway_left" / "sway_right"
        self.gate_sway_start_time = None
        self._search_sway_count = 0
        self._gate_handling_active = False

        self._post_flare_gate_scan = False

        # ── SEARCH FLARE ─────────────────────────────────────────────────
        self.search_state = None  # "SCAN"
        self.hit_confirmed = False
        self.hit_start_time = None
        self._camera_yaw_lost_start = None

        # ── SEARCH COLOR FLARE sub-state ──────────────────────────────────
        self._search_timeout_state  = None   # None / "SWAY"
        self._search_timeout_start  = None
        self._search_scan_start     = None
        self._search_sway_direction = None   # "sway_left" / "sway_right"
        self._last_dodge_direction  = "camera_sway_forward_left"  # default

        # BUCKET TRACKING
        self.bucket_detected_once = False

        # BUCKET DETECTION
        self.drop_ball_status = 0

        # ── SCAN ──────────────────────────────────────────────────────────
        self.base_yaw   = 261.0
        self.sensor_yaw = self.base_yaw
        self.scan_angle = 140.0
        self.scan_left  = True
        self.scan_target_yaw = self.base_yaw
        self.scan_step = .9

        # ── SETPOINT ──────────────────────────────────────────────────────
        self.set_point       = SetPoint()
        self.set_point.roll  = 0.0
        self.set_point.pitch = 0.0
        self.set_point.yaw   = self.base_yaw
        self.set_point.yaw = self.set_point.yaw % 360
        self.set_point.depth = 0.7


        # ── PID ───────────────────────────────────────────────────────────
        self.multi_pid = MultiPID()

        pid_yaw         = PID(); pid_yaw.kp   = 4.5;    pid_yaw.kd   = 0.3
        pid_pitch       = PID(); pid_pitch.kp = 9.0;   pid_pitch.kd = 1.7
        pid_roll        = PID(); pid_roll.kp  = 2.5;    pid_roll.kd  = 0.4
        pid_depth       = PID(); pid_depth.kp = 1350.0; pid_depth.kd = 215.0
        pid_camera      = PID(); pid_camera.kp = 0.18

        self.multi_pid.pid_yaw    = pid_yaw
        self.multi_pid.pid_pitch  = pid_pitch
        self.multi_pid.pid_roll   = pid_roll
        self.multi_pid.pid_depth  = pid_depth
        self.multi_pid.pid_camera = pid_camera

        # ── PUBLISHERS ────────────────────────────────────────────────────
        self.pub_set_point = self.create_publisher(SetPoint, "set_point", 10)
        self.pub_pid       = self.create_publisher(MultiPID, "pid",       10)
        self.pub_status    = self.create_publisher(String,   "status",    10)

        # ── SUBSCRIBER ────────────────────────────────────────────────────
        self.create_subscription(
            ObjectDifference,
            'object_difference',
            self.object_difference_callback,
            10
        )

        self.create_subscription(
            Sensor,
            'sensor_msg',
            self.sensor_callback,
            10
        )

        self.create_subscription(
            Float32,
            'drop_ball_msg',
            self.drop_ball_callback,
            10
        )

        self.pub_pid.publish(self.multi_pid)

        # ── MAIN LOOP 10 Hz ───────────────────────────────────────────────
        self.create_timer(0.1, self.loop)

    # ═══════════════════════════════════════════════════════════════════════
    # UTILS
    # ═══════════════════════════════════════════════════════════════════════

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.now() - self.state_start_time

    def change_state(self, new_state):
        self.state            = new_state
        self.state_start_time = self.now()
        self.get_logger().info(f"STATE → {new_state}")

    def publish_status(self, status_str):
        msg      = String()
        msg.data = status_str
        self.pub_status.publish(msg)

    def do_scan(self):
        # """Scan berbasis sensor_yaw — lebih presisi."""
        # SCAN_RANGE = 120.0  # derajat kiri-kanan dari base_yaw
        
        # diff = self.sensor_yaw - self.base_yaw
        # if diff > 180:  diff -= 360
        # if diff < -180: diff += 360
        
        # if self.scan_left:
        #     if diff > SCAN_RANGE:   # sudah terlalu kiri
        #         self.scan_left = False
        #     self.publish_status("yaw_left")
        # else:
        #     if diff < -SCAN_RANGE:  # sudah terlalu kanan
        #         self.scan_left = True
        #     self.publish_status("yaw_right")
        self.publish_status("yaw_left")
        

    # ═══════════════════════════════════════════════════════════════════════
    # SUBSCRIBER CALLBACK
    # ═══════════════════════════════════════════════════════════════════════

    def object_difference_callback(self, msg):
        self.object_class = msg.object_type
        self.is_target    = msg.is_target
        self.x_difference  = msg.x_difference
        # x_difference tidak dipakai di sini —
        # teensy yang konsumsi langsung dari topic object_difference

    def sensor_callback(self, msg):
        self.sensor_yaw = msg.yaw
    
    def drop_ball_callback(self, msg):
        self.drop_ball_status = msg.data

    # ═══════════════════════════════════════════════════════════════════════
    # MAIN LOOP
    # ═══════════════════════════════════════════════════════════════════════

    def loop(self):
        if self.start_time is None:
            self.start_time = self.now()

        boot_time = self.now() - self.start_time

        if boot_time < self.param_delay:
            remaining = self.param_delay - boot_time
            self.get_logger().info(f'DELAY... sisa {remaining:.1f}s')
            return

        if self.state == "INIT":
            self.change_state("STABILIZE")

        self.start_auv()

    # ═══════════════════════════════════════════════════════════════════════
    # MISSION STATES
    # ═══════════════════════════════════════════════════════════════════════

    def start_auv(self):

        # ─── STABILIZE ───────────────────────────────────────────────────
        if self.state == "STABILIZE":
            self.pub_set_point.publish(self.set_point)
            self.publish_status("dpr_ssy")

            if self.elapsed() > 2:
                self.change_state("SEARCH_ORANGE_FLARE")

        # ─── SEARCH ORANGE FLARE ─────────────────────────────────────────
        # elif self.state == "SEARCH_ORANGE_FLARE":

        #     if self.object_class == "orange_flare":

        #         if self.flare_lock_start is None:
        #             self.flare_lock_start = self.now()

        #         # Teensy handle centering via x_difference + status "camera"
        #         self.publish_status("camera")

        #         lock_time = self.now() - self.flare_lock_start
        #         if lock_time > 0.5:
        #             self.get_logger().info("ORANGE FLARE LOCKED")
        #             self.change_state("DODGE_ORANGE_FLARE")

        #     else:
        #         self.flare_lock_start = None

        #         if self.elapsed() <= 2:
        #             self.get_logger().info("ORANGE FLARE NOT FOUND → FORWARD (NO YAW)")
        #             self.publish_status("last_no_yaw")
        #         else:
        #             self.get_logger().info("ORANGE FLARE NOT FOUND → FORWARD (YAW)")
        #             self.last_scan_time = None
        #             self.set_point.yaw = self.base_yaw
        #             self.pub_set_point.publish(self.set_point)
        #             self.publish_status("all")

        # ─── SEARCH ORANGE FLARE ─────────────────────────────────────────
        elif self.state == "SEARCH_ORANGE_FLARE":
            if self.orange_search_start is None:
                self.orange_search_start = self.now()

            search_time = self.now() - self.orange_search_start

            if search_time > 7.0 and not self.flare_orange_detected_once:
                self.get_logger().info("ORANGE FLARE NOT FOUND 13s → SKIP")
                self.orange_search_start = None
                self.change_state("DODGE_ORANGE_FLARE")
                return
            
            if self.object_class == "orange_flare" :
                # terlihat, bukan is_target
                self.flare_orange_detected_once = True

                # Cek is_target once
                if self.is_target and not self.flare_orange_is_target_once:
                    self.flare_orange_is_target_once = True
                    self.get_logger().info("ORANGE FLARE IS_TARGET ONCE → DPR_SSY")

                if self.flare_orange_is_target_once:

                    if self.flare_lock_start is None:
                        self.flare_lock_start = self.now()

                    self.publish_status("dpr")

                    lock_time = self.now() - self.flare_lock_start
                    if lock_time > 1:
                        self.get_logger().info(f"ORANGE FLARE LOCKED {lock_time:.1f}s → DODGE")
                        self.flare_orange_is_target_once = False
                        self.flare_lock_start = None
                        self.orange_search_start = None
                        self.change_state("DODGE_ORANGE_FLARE")
                else:
                    # Terlihat tapi belum is_target → tetap maju centering
                    self.publish_status("camera_yaw")

            else:
                if self.flare_orange_is_target_once:
                    # Pernah is_target → tetap diam meski flare hilang
                    self.publish_status("dpr")
                else:
                    # Belum pernah is_target → cari
                    if self.elapsed() <= 2:
                        self.publish_status("all")
                    else:
                        self.set_point.yaw = self.base_yaw
                        self.pub_set_point.publish(self.set_point)
                        self.publish_status("all")

        # ─── DODGE ORANGE FLARE ──────────────────────────────────────────
        elif self.state == "DODGE_ORANGE_FLARE":
            if not hasattr(self, 'no_orange_seen') or self.no_orange_seen is None:
                self.no_orange_seen = self.now()

            if self.x_difference >= 100:
                self.publish_status("camera_sway_forward_left")
                self._last_dodge_direction = "camera_sway_forward_left"
            elif self.x_difference <= -100:
                self.publish_status("camera_sway_forward_right")
                self._last_dodge_direction = "camera_sway_forward_right"
            else:
                self.publish_status("camera_sway_forward_left")
                self._last_dodge_direction = "camera_sway_forward_left"

            if self.object_class == "orange_flare":
                self.last_orange_flare_seen     = self.now()
                self.flare_orange_detected_once = True
            else:
                if self.flare_orange_detected_once:
                    lost_time = self.now() - self.last_orange_flare_seen
                    if lost_time > 4:
                        self.get_logger().info("ORANGE FLARE DODGED")
                        self.change_state("COLOR_FLARE_CHALLENGE")
                else:
                    lost_time_no_orange_seen = self.now() - self.no_orange_seen
                    if lost_time_no_orange_seen > 4:
                        self.get_logger().info("ORANGE FLARE DODGED")
                        self.change_state("COLOR_FLARE_CHALLENGE")

        # ─── COLOR FLARE CHALLENGE ────────────────────────────────────────
        elif self.state == "COLOR_FLARE_CHALLENGE":
            self.set_point.depth = -0.17
            self.pub_set_point.publish(self.set_point)
            self._handle_color_flare_challenge()

        # ─── SEARCH GATE ─────────────────────────────────────────────────
        elif self.state == "SEARCH_GATE":
            self.set_point.depth = 0.0
            self.pub_set_point.publish(self.set_point)

            if not hasattr(self, 'no_gate_seen') or self.no_gate_seen is None:
                self.no_gate_seen = self.now()

            if self.object_class == "gate":
                # Teensy handle centering + maju via x_difference + status "camera"
                self.publish_status("camera")
                self.last_gate_seen     = self.now()
                self.gate_detected_once = True
                self.get_logger().info("GATE DETECTED")
            else:
                if self.gate_detected_once:
                    lost_time = self.now() - self.last_gate_seen
                    if lost_time > 10:
                        self.get_logger().info("GATE PASSED")
                        self.change_state("SEARCH_BUCKET")
                else:
                    if not self.gate_detected_once:
                        lost_time_no_gate_seen = self.now() - self.no_gate_seen

                        if lost_time_no_gate_seen > 10:
                            self.get_logger().info("GATE NOT PASSED")
                            self.change_state("SURFACE")
                        else:
                            # ── BELUM PERNAH DETECT GATE → LAWAN ARAH DODGE ──
                            if self._last_dodge_direction == "camera_sway_forward_left":
                                self.publish_status("sway_left_forward")
                            else:
                                self.publish_status("sway_right_forward")

        # ─── SEARCH BUCKET ─────────────────────────────────────────────────
        elif self.state == "SEARCH_BUCKET":
            self.set_point.depth = -0.2
            self.pub_set_point.publish(self.set_point)

            if not hasattr(self, 'no_bucket_seen') or self.no_bucket_seen is None:
                self.no_bucket_seen = self.now()

            if self.object_class == "blue_bucket":
                self.publish_status("camera_slow")
                self.get_logger().info("BUCKET DETECTED")
                self.change_state("GO_BUCKET")
            else:
                lost_time = self.now() - self.no_bucket_seen

                if lost_time > 11.5:
                    self.get_logger().info("BUCKET NOT FOUND, SURFACE")
                    self.change_state("SURFACE")
                else :
                    self.publish_status("yaw_left")
        
        # GO TO BUCKET
        elif self.state == "GO_BUCKET":
            if self.object_class == "blue_bucket":
                self.get_logger().info("BUCKET LOCKED")
                self.last_bucket_seen = self.now()
                self.bucket_detected_once = True

            else:
                if self.bucket_detected_once:

                    lost_time = self.now() - self.last_bucket_seen

                    if lost_time >= 1 and self.drop_ball_status == 1.0:
                        self.get_logger().info("BUCKET PASSED")
                        self.change_state("SURFACE")
                    elif lost_time >= 5:
                        self.get_logger().info("BUCKET NO PASSED")
                        self.change_state("SURFACE")

        # ─── SURFACE ─────────────────────────────────────────────────────
        elif self.state == "SURFACE":
            self.set_point.depth = -1.0
            self.pub_set_point.publish(self.set_point)
            self.publish_status("dpr_ssy")

    # ═══════════════════════════════════════════════════════════════════════
    # COLOR FLARE CHALLENGE HANDLER
    # ═══════════════════════════════════════════════════════════════════════

    def _handle_color_flare_challenge(self):
        """
        Sub-state machine menabrak 3 flare warna (random order).
        SEARCH → APPROACH → BACK → (flare berikutnya) → SEARCH_GATE
        """
        remaining = COLOR_FLARES - self.color_flares_done

        if not remaining:
            self.get_logger().info("ALL COLOR FLARES HIT → SEARCH_GATE")
            self.change_state("SEARCH_GATE")
            return

        if self.color_flare_state == "SEARCH":
            if self.object_class in remaining:
                self.current_color_flare  = self.object_class
                self.color_flare_state    = "APPROACH"
                self.color_flare_hit_time = None
                self.color_flare_locked   = True
                # reset search sub-state
                self._search_timeout_state = None
                self._search_timeout_start = None
                self._search_scan_start    = None
                self._post_flare_gate_scan  = False
                self.get_logger().info(f"COLOR FLARE FOUND: {self.current_color_flare}")
            
            elif self._post_flare_gate_scan:
                if self.object_class == "gate" or self._gate_handling_active:
                    self._post_flare_gate_scan = False
                    self._gate_handling_active = True
                    self.gate_sway_state     = None   # ← tambah ini
                    self.gate_scan_start_time = None  # ← tambah ini
                    self._handle_gate_in_search()
                else:
                    self.do_scan()  # scan saja, tidak ada timeout ke sway

            # ── Gate handling: sekali aktif, tidak bisa keluar sampai selesai ──
            elif self._gate_handling_active or (
                self.object_class == "gate"
                and self._search_timeout_state != "SWAY"
                and self.gate_sway_state is None
            ):
                if not self._gate_handling_active:
                    self._gate_handling_active = True  # kunci di sini
                    self.get_logger().info("GATE DETECTED → lock ke _handle_gate_in_search")
                self._handle_gate_in_search()

            else:
                # ── Tidak detect apa-apa (atau gate saat sway) ───────────

                if self._search_scan_start is None:
                    self._search_scan_start = self.now()

                # ── YAW_ESCAPE phase (setelah count >= 3) ─────────────────
                if self._search_timeout_state == "YAW_ESCAPE":
                    yaw_elapsed = self.now() - self._search_timeout_start

                    if yaw_elapsed < 1.0:
                        # Luruskan dulu ke base_yaw
                        self.set_point.yaw = self.base_yaw
                        self.pub_set_point.publish(self.set_point)
                        self.publish_status("dpr_ssy")

                    else:
                        self.get_logger().info("FORWARD_ESCAPE done → SEARCH_GATE")
                        self._search_timeout_state  = None
                        self._search_timeout_start  = None
                        self._search_scan_start     = None
                        self._search_sway_direction = None
                        self._search_sway_count     = 0
                        self.gate_sway_state        = None
                        self.gate_scan_start_time   = None
                        self._gate_handling_active  = False
                        self.change_state("SEARCH_GATE")
                    return

                # ── FORWARD_ESCAPE phase (maju setelah yaw) ───────────────
                # if self._search_timeout_state == "FORWARD_ESCAPE":
                #     fwd_elapsed = self.now() - self._search_timeout_start

                #     if fwd_elapsed < 4.0:
                #         self.publish_status("all")  # maju lurus
                #     else:
                #         # Selesai maju → paksa _handle_gate_in_search
                #         self.get_logger().info("FORWARD_ESCAPE done → SEARCH_GATE")
                #         self._search_timeout_state  = None
                #         self._search_timeout_start  = None
                #         self._search_scan_start     = None
                #         self._search_sway_direction = None
                #         self._search_sway_count     = 0
                #         self.gate_sway_state        = None
                #         self.gate_scan_start_time   = None
                #         self._gate_handling_active  = False
                #         self.change_state("SEARCH_GATE")
                #     return

                # ── SWAY phase (setelah timeout scan) ────────────────────
                if self._search_timeout_state == "SWAY":
                    sway_elapsed = self.now() - self._search_timeout_start

                    if sway_elapsed < 1.0:
                        self.set_point.yaw = self.base_yaw
                        self.pub_set_point.publish(self.set_point)
                        self.publish_status("dpr_ssy")

                    elif sway_elapsed < 10.0:
                        if self.object_class == "gate" and self._search_sway_count >= 2:
                            self.get_logger().info(
                                f"GATE DETECTED saat SWAY ke-{self._search_sway_count + 1} → _handle_gate_in_search"
                            )
                            self._search_timeout_state  = None
                            self._search_timeout_start  = None
                            self._search_scan_start     = None
                            self._search_sway_direction = None
                            self.gate_sway_state        = None
                            self.gate_scan_start_time   = None
                            self._handle_gate_in_search()
                            return

                        # ── CEK COUNT DI SINI JUGA (bukan hanya di SCAN) ──
                        if self._search_sway_count >= 1:
                            self.get_logger().info(
                                f"SWAY count {self._search_sway_count} ≥ 1 saat SWAY → YAW_ESCAPE"
                            )
                            # Tentukan arah yaw: lawan arah dodge terakhir
                            self._search_sway_direction = (
                                "sway_right"
                                if self._last_dodge_direction == "camera_sway_forward_left"
                                else "sway_left"
                            )
                            self._search_timeout_state = "YAW_ESCAPE"
                            self._search_timeout_start = self.now()
                            return

                        self.publish_status(self._search_sway_direction)

                    else:
                        self.get_logger().info(
                            f"SWAY {self._search_sway_direction} DONE → SCAN lagi"
                        )
                        self._search_sway_count += 1

                        # ── CEK COUNT SETELAH INCREMENT ──
                        if self._search_sway_count >= 1:
                            self.get_logger().info(
                                f"SWAY count {self._search_sway_count} ≥ 1 setelah selesai → YAW_ESCAPE"
                            )
                            self._search_sway_direction = (
                                "sway_right"
                                if self._last_dodge_direction == "camera_sway_forward_left"
                                else "sway_left"
                            )
                            self._search_timeout_state = "YAW_ESCAPE"
                            self._search_timeout_start = self.now()
                            return

                        self._search_sway_direction = (
                            "sway_left"
                            if self._search_sway_direction == "sway_right"
                            else "sway_right"
                        )
                        self._search_timeout_state = None
                        self._search_scan_start    = self.now()

                # ── SCAN phase ────────────────────────────────────────────
                else:
                    # Tidak perlu cek count di sini lagi — sudah dicek di SWAY
                    self.do_scan()

                    scan_elapsed = self.now() - self._search_scan_start
                    if scan_elapsed > 20.0:
                        if self._search_sway_direction is None:
                            self._search_sway_direction = (
                                "sway_right"
                                if self._last_dodge_direction == "camera_sway_forward_left"
                                else "sway_left"
                            )
                        self.get_logger().info(
                            f"SCAN TIMEOUT 20s → SWAY {self._search_sway_direction}"
                        )
                        self._search_timeout_state = "SWAY"
                        self._search_timeout_start = self.now()

        elif self.color_flare_state == "APPROACH":
            # ── Sub-state tracker ──────────────────────────────────────────
            # self._approach_phase: None → "CAMERA_YAW" → "FORWARD"
            if not hasattr(self, '_approach_phase'):
                self._approach_phase = None
            if not hasattr(self, '_is_target_once'):
                self._is_target_once = False
            if not hasattr(self, '_camera_yaw_start'):
                self._camera_yaw_start = None
            if not hasattr(self, '_forward_start'):
                self._forward_start = None
            if not hasattr(self, '_locked_yaw'):
                self._locked_yaw = None

            # ── PHASE: FORWARD (maju 3 detik pakai yaw yang sudah dikunci) ─
            if self._approach_phase == "FORWARD":
                self.publish_status("all")

                if self.now() - self._forward_start > 3.0:
                    self.get_logger().info("FORWARD 3s DONE → BACK")
                    self._substate_start   = self.now()
                    self.color_flare_state = "BACK"
                    # reset semua approach state
                    self._approach_phase   = None
                    self._is_target_once   = False
                    self._camera_yaw_start = None
                    self._forward_start    = None
                    self._locked_yaw       = None
                return

            # ── PHASE: CAMERA_YAW (diam, luruskan yaw, validasi 1.5 detik) ─
            if self._approach_phase == "CAMERA_YAW":
                self.publish_status("camera_yaw")

                # Validasi: pastikan flare masih sama selama 1.5 detik
                if self.object_class == self.current_color_flare:
                    self._camera_yaw_lost_start = None  # reset timer hilang
                    # Reset timer validasi kalau sempat hilang
                    if self._camera_yaw_start is None:
                        self._camera_yaw_start = self.now()

                    elapsed_val = self.now() - self._camera_yaw_start

                    # Kunci yaw saat x_difference == 0
                    if self.x_difference <= 10 and self.x_difference >= -10 and self._locked_yaw is None:
                        self._locked_yaw          = self.sensor_yaw
                        self.set_point.yaw        = self._locked_yaw
                        self.pub_set_point.publish(self.set_point)
                        self.get_logger().info(f"YAW LOCKED: {self._locked_yaw:.1f}°")

                    # Validasi 1.5 detik selesai → FORWARD
                    if elapsed_val > .5 and self._locked_yaw is not None:
                        self.get_logger().info("CAMERA_YAW VALIDATED → FORWARD")
                        self._approach_phase = "FORWARD"
                        self._forward_start  = self.now()

                else:
                    self._camera_yaw_start = None  # reset timer terlihat   
                    # Flare hilang — mulai hitung dari sekarang kalau belum
                    if self._camera_yaw_lost_start is None:
                        self._camera_yaw_lost_start = self.now()


                    lost_elapsed = self.now() - self._camera_yaw_lost_start
                    if lost_elapsed > 1:
                        self.get_logger().warn(
                            f"CAMERA_YAW: {self.current_color_flare} hilang 1s → SEARCH"
                        )
                        self.color_flare_state   = "SEARCH"
                        self.current_color_flare = None
                        self._approach_phase     = None
                        self._is_target_once     = False
                        self._camera_yaw_start   = None
                        self._locked_yaw         = None
                        self._camera_yaw_lost_start = None
                return

            # ── PHASE: CAMERA (maju centering, tunggu is_target sekali) ────
            self.publish_status("camera")

            if self.object_class == self.current_color_flare:
                self.color_flare_locked = True
                if self.is_target and not self._is_target_once:
                    self._is_target_once   = True
                    self._approach_phase   = "CAMERA_YAW"
                    self._camera_yaw_start = None  # mulai fresh di CAMERA_YAW
                    self.get_logger().info(
                        f"IS_TARGET ONCE: {self.current_color_flare} → CAMERA_YAW"
                    )
            
            elif self.object_class in (COLOR_FLARES - self.color_flares_done):
                # Flare BERGANTI ke warna lain → langsung pindah target
                self.get_logger().warn(
                    f"FLARE BERGANTI: {self.current_color_flare} → {self.object_class}, reset SEARCH"
                )
                self.color_flare_state   = "SEARCH"
                self.current_color_flare = None
                self._approach_phase     = None
                self._is_target_once     = False
                self.color_flare_locked  = False


            elif not self.color_flare_locked:
                # Flare hilang dan belum pernah locked → SEARCH
                self.color_flare_state   = "SEARCH"
                self.current_color_flare = None
                self._approach_phase     = None
                self._is_target_once     = False
                self.get_logger().warn("COLOR FLARE LOST sebelum locked → SEARCH")

        # ── BACK: mundur 3 detik ──────────────────────────────────────────
        elif self.color_flare_state == "BACK":
            self.publish_status("backward_yaw")

            if self.now() - self._substate_start > 2.0:
                self.color_flares_done.add(self.current_color_flare)
                self.get_logger().info(
                    f"{self.current_color_flare} DONE | "
                    f"remaining: {COLOR_FLARES - self.color_flares_done}"
                )
                self.current_color_flare  = None
                self.color_flare_hit_time = None
                self._substate_start      = None
                self._is_target_since     = None
                self.color_flare_locked   = False  # ← tambah ini
                self.color_flare_state    = "SEARCH"
                self.state_start_time     = self.now()  # reset scan timer

                # ── reset search sub-state untuk flare berikutnya ──
                self._search_timeout_state  = None
                self._search_timeout_start  = None
                self._search_scan_start     = None
                # self._search_sway_direction = None
    
    # HANDLER GATE DI SEARCH: kalau gate terdeteksi saat scan, langsung align + sway ke arah gate, scan lagi untuk cari color flare, kalau timeout cari gate lagi lalu masuk SEARCH_GATE
    def _handle_gate_in_search(self):

        # ── ALIGN: luruskan ke base_yaw dulu ─────────────────────────
        if self.gate_sway_state is None:
            diff = self.sensor_yaw - self.base_yaw
            if diff > 180:  diff -= 360
            if diff < -180: diff += 360

            self.gate_sway_direction = "sway_left" if diff < 0 else "sway_right"
            self.get_logger().info(
                f"GATE di {'kiri' if diff < 0 else 'kanan'} "
                f"(sensor_yaw={self.sensor_yaw:.1f}, base={self.base_yaw}) "
                f"→ ALIGN dulu ke base_yaw, lalu {self.gate_sway_direction}"
            )

            # Luruskan yaw ke base dulu, tunggu sampai mendekati base_yaw
            self.set_point.yaw = self.base_yaw
            self.pub_set_point.publish(self.set_point)
            self.gate_sway_state      = "ALIGN"
            self.gate_sway_start_time = self.now()
            return

        # ── ALIGN: tunggu yaw sudah lurus ke base_yaw ────────────────
        if self.gate_sway_state == "ALIGN":
            self.set_point.yaw = self.base_yaw
            self.pub_set_point.publish(self.set_point)
            self.publish_status("dpr_ssy")

            diff = self.sensor_yaw - self.base_yaw
            if diff > 180:  diff -= 360
            if diff < -180: diff += 360

            # Toleransi ±5 derajat dari base_yaw → anggap sudah lurus
            if abs(diff) <= 5:
                self.get_logger().info(
                    f"ALIGN DONE (sensor_yaw={self.sensor_yaw:.1f}) → SWAY"
                )
                self.gate_sway_state      = "SWAY"
                self.gate_sway_start_time = self.now()
            return

        # ── SWAY: gerak ke arah gate sampai gate terdeteksi ──────────
        if self.gate_sway_state == "SWAY":
            if self.now() - self.gate_sway_start_time > 7.0:
                # Timeout 10 detik tidak ketemu gate → langsung SCAN
                self.get_logger().info("SWAY TIMEOUT 7s → SCAN")
                self.gate_sway_state      = "SCAN"
                self.gate_scan_start_time = self.now()
                return
            
            if self.object_class == "gate":
                # Gate terdeteksi → centering pakai camera_sway
                self.publish_status("camera_sway")

                if -10 <= self.x_difference <= 10:
                    self.get_logger().info("GATE CENTER → mulai SCAN")
                    self.gate_sway_state      = "SCAN"
                    self.gate_scan_start_time = self.now()
            else:
                # Belum ketemu gate → gerak buta ke arah yang ditentukan
                self.publish_status(self.gate_sway_direction)
            return

        # ── SCAN: cari color flare, abaikan gate, timeout 10 detik ───
        elif self.gate_sway_state == "SCAN":
            self.do_scan()

            # Gate diabaikan sepenuhnya saat SCAN
            if self.object_class == "gate":
                self.get_logger().info("SCAN: gate terdeteksi, diabaikan")
                return

            if self.object_class in (COLOR_FLARES - self.color_flares_done):
                self.gate_sway_state        = None
                self.gate_scan_start_time   = None
                self._gate_handling_active  = False
                self._post_flare_gate_scan  = True   # ← tandai: habis ini scan-only
                self.get_logger().info(f"SCAN: flare ditemukan → APPROACH, lalu scan-only")

                return

            if self.now() - self.gate_scan_start_time > 10.0:
                self.get_logger().info("SCAN TIMEOUT 10s → FIND_GATE_THEN_GO")
                self.gate_sway_state = "FIND_GATE_THEN_GO"
            return

        # ── FIND_GATE_THEN_GO: scan sampai ketemu gate → SEARCH_GATE ─
        elif self.gate_sway_state == "FIND_GATE_THEN_GO":
            self.change_state("SEARCH_GATE")
            if self.object_class == "gate":
                self.get_logger().info("GATE FOUND after timeout → SEARCH_GATE")
                self.gate_sway_state      = None
                self.gate_scan_start_time = None
                self._gate_handling_active = False
            else:
                self.do_scan()


# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = Guidance()
    rclpy.spin(node)
    node.destroy_node()
    node.publish_status("stop")
    rclpy.shutdown()


if __name__ == "__main__":
    main()