#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDifference


COLOR_FLARES = {"red_flare", "yellow_flare", "blue_flare"}


class Guidance(Node):

    def __init__(self):
        super().__init__('guidance')

        # ── TIME ──────────────────────────────────────────────────────────
        self.start_time       = None
        self.state_start_time = None

        # ── STATE ─────────────────────────────────────────────────────────
        self.state = "INIT"

        # ── OBJECT INFO (dari ObjectDifference) ───────────────────────────
        self.object_class = ""
        self.is_target    = False

        # ── ORANGE FLARE SCAN ─────────────────────────────────────────────
        self.scan_left      = True
        self.last_scan_time = None
        self.scan_dwell     = 2.0 # detik tahan di tiap sisi

        # ── ORANGE FLARE TRACKING ─────────────────────────────────────────
        self.flare_lock_start           = None
        self.flare_orange_detected_once = False
        self.last_orange_flare_seen     = 0

        # ── COLOR FLARE CHALLENGE ─────────────────────────────────────────
        self.color_flares_done    = set()
        self.current_color_flare  = None
        self.color_flare_hit_time = None        # kapan is_target pertama True
        self.color_flare_state    = "SEARCH"    # sub-state: SEARCH / APPROACH / BACK
        self._substate_start      = None

        # ── GATE TRACKING ─────────────────────────────────────────────────
        self.gate_detected_once = False
        self.last_gate_seen     = 0

        # ── SCAN ──────────────────────────────────────────────────────────
        self.base_yaw   = 270.0
        self.scan_angle = 30.0
        self.scan_left  = True

        # ── SETPOINT ──────────────────────────────────────────────────────
        self.set_point       = SetPoint()
        self.set_point.roll  = 0.0
        self.set_point.pitch = 0.0
        self.set_point.yaw   = self.base_yaw
        self.set_point.depth = -0.3

        # ── PID ───────────────────────────────────────────────────────────
        self.multi_pid = MultiPID()

        pid_yaw         = PID(); pid_yaw.kp   = 4.5;    pid_yaw.kd   = 0.3
        pid_pitch       = PID(); pid_pitch.kp = 15.0;   pid_pitch.kd = 2.6
        pid_roll        = PID(); pid_roll.kp  = 2.5;    pid_roll.kd  = 0.4
        pid_depth       = PID(); pid_depth.kp = 1350.0; pid_depth.kd = 215.0
        pid_camera      = PID(); pid_camera.kp = 1.0

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
        """Scan kiri-kanan setiap 2 detik."""
        now = self.now()

        if self.last_scan_time is None or now - self.last_scan_time > self.scan_dwell:
            self.last_scan_time = now
            self.scan_left = not self.scan_left

        self.set_point.yaw    = self.base_yaw - self.scan_angle if self.scan_left else self.base_yaw + self.scan_angle
        self.pub_set_point.publish(self.set_point)

    # ═══════════════════════════════════════════════════════════════════════
    # SUBSCRIBER CALLBACK
    # ═══════════════════════════════════════════════════════════════════════

    def object_difference_callback(self, msg):
        self.object_class = msg.object_type
        self.is_target    = msg.is_target
        # x_difference tidak dipakai di sini —
        # teensy yang konsumsi langsung dari topic object_difference

    # ═══════════════════════════════════════════════════════════════════════
    # MAIN LOOP
    # ═══════════════════════════════════════════════════════════════════════

    def loop(self):
        if self.start_time is None:
            self.start_time = self.now()
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
        elif self.state == "SEARCH_ORANGE_FLARE":

            if self.object_class == "orange_flare":

                if self.flare_lock_start is None:
                    self.flare_lock_start = self.now()

                # Teensy handle centering via x_difference + status "camera"
                self.publish_status("camera")

                lock_time = self.now() - self.flare_lock_start
                if lock_time > 1.5:
                    self.get_logger().info("ORANGE FLARE LOCKED")
                    self.change_state("DODGE_ORANGE_FLARE")

            else:
                self.flare_lock_start = None

                if self.elapsed() <= 5:
                    self.get_logger().info("ORANGE FLARE NOT FOUND → SCAN")
                    self.do_scan()
                else:
                    self.get_logger().info("ORANGE FLARE NOT FOUND → FORWARD")
                    self.last_scan_time = None
                    self.set_point.yaw = self.base_yaw
                    self.pub_set_point.publish(self.set_point)
                    self.publish_status("all")

        # ─── DODGE ORANGE FLARE ──────────────────────────────────────────
        elif self.state == "DODGE_ORANGE_FLARE":
            self.publish_status("sway_right_forward")

            if self.object_class == "orange_flare":
                self.last_orange_flare_seen     = self.now()
                self.flare_orange_detected_once = True
            else:
                if self.flare_orange_detected_once:
                    lost_time = self.now() - self.last_orange_flare_seen
                    if lost_time > 2:
                        self.get_logger().info("ORANGE FLARE DODGED")
                        self.change_state("COLOR_FLARE_CHALLENGE")

        # ─── COLOR FLARE CHALLENGE ────────────────────────────────────────
        elif self.state == "COLOR_FLARE_CHALLENGE":
            self._handle_color_flare_challenge()

        # ─── SEARCH GATE ─────────────────────────────────────────────────
        elif self.state == "SEARCH_GATE":

            if self.object_class == "Gate":
                # Teensy handle centering + maju via x_difference + status "camera"
                self.publish_status("camera")
                self.last_gate_seen     = self.now()
                self.gate_detected_once = True
                self.get_logger().info("GATE DETECTED")
            else:
                self.publish_status("dpr_ssy")
                if self.gate_detected_once:
                    lost_time = self.now() - self.last_gate_seen
                    if lost_time > 4:
                        self.get_logger().info("GATE PASSED")
                        self.change_state("SURFACE")

        # ─── SURFACE ─────────────────────────────────────────────────────
        elif self.state == "SURFACE":
            self.set_point.depth = 0.0
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

        # ── SEARCH ───────────────────────────────────────────────────────
        if self.color_flare_state == "SEARCH":
            self.publish_status("dpr_ssy")

            if self.object_class in remaining:
                self.current_color_flare = self.object_class
                self.color_flare_state   = "APPROACH"
                self.color_flare_hit_time = None
                self.get_logger().info(
                    f"COLOR FLARE FOUND: {self.current_color_flare}"
                )
            else:
                self.do_scan()

        # ── APPROACH: status "camera" → teensy maju + centering ──────────
        # Tunggu is_target True (bbox penuh), dwell 1.5 detik lalu BACK
        elif self.color_flare_state == "APPROACH":
            self.publish_status("camera")

            if self.object_class != self.current_color_flare:
                # Flare hilang dari frame → balik cari lagi
                self.color_flare_state    = "SEARCH"
                self.current_color_flare  = None
                self.color_flare_hit_time = None
                self.get_logger().warn("COLOR FLARE LOST → SEARCH")
                return

            if self.is_target:
                if self.color_flare_hit_time is None:
                    self.color_flare_hit_time = self.now()
                    self.get_logger().info(
                        f"FULL FRAME: {self.current_color_flare}, dwell 1.5s"
                    )

                if self.now() - self.color_flare_hit_time > 1.5:
                    self._substate_start   = self.now()
                    self.color_flare_state = "BACK"
                    self.get_logger().info("DWELL DONE → BACK")
            else:
                self.color_flare_hit_time = None  # belum penuh, reset timer

        # ── BACK: mundur 3 detik ──────────────────────────────────────────
        elif self.color_flare_state == "BACK":
            self.publish_status("backward")

            if self.now() - self._substate_start > 3.0:
                self.color_flares_done.add(self.current_color_flare)
                self.get_logger().info(
                    f"{self.current_color_flare} DONE | "
                    f"remaining: {COLOR_FLARES - self.color_flares_done}"
                )
                self.current_color_flare  = None
                self.color_flare_hit_time = None
                self._substate_start      = None
                self.color_flare_state    = "SEARCH"
                self.state_start_time     = self.now()  # reset scan timer


# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = Guidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()