#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDetection, ObjectDifference


class Guidance(Node):

    def __init__(self):
        super().__init__('guidance')

        # TIME
        self.start_time = None
        self.state_start_time = None

        # DELAY
        self.param_delay = 5.0  # ← ganti nilai ini sesuai kebutuhan (detik)

        # STATE
        self.state = "INIT"

        # OBJECT DETECTION
        self.object_class = ""

        # SCAN
        self.base_yaw = 261.0
        self.scan_angle = 30.0
        self.scan_left = True

        # FLARE LOCK AND LOST
        self.flare_lock_start = None
        self.flare_orange_is_target_once   = False  # ← untuk SEARCH: sudah pernah is_target
        self.flare_orange_detected_once = False
        self.last_orange_flare_seen = 0

        # GATE TRACKING
        self.gate_detected_once = False
        self.last_gate_seen = 0

        # DODGE ORANGE FLARE
        self.status_dodge_flare = "camera_sway_right_forward"

        # BUCKET TRACKING
        self.bucket_detected_once = False

        # SETPOINT
        self.set_point = SetPoint()
        self.set_point.roll = 0.0
        self.set_point.pitch = 0.0
        self.set_point.yaw = self.base_yaw
        self.set_point.depth = 0.7

        # PID
        self.multi_pid = MultiPID()

        pid_yaw = PID()
        pid_yaw.kp = 4.5
        pid_yaw.kd = 0.3

        pid_pitch = PID()
        pid_pitch.kp = 9.0
        pid_pitch.kd = 1.7

        pid_roll = PID()
        pid_roll.kp = 3.0
        pid_roll.kd = 0.3

        pid_depth = PID()
        pid_depth.kp = 1350.0
        pid_depth.kd = 215.0

        pid_camera = PID()
        pid_camera.kp = 0.12

        self.multi_pid.pid_yaw = pid_yaw
        self.multi_pid.pid_pitch = pid_pitch
        self.multi_pid.pid_roll = pid_roll
        self.multi_pid.pid_depth = pid_depth
        self.multi_pid.pid_camera = pid_camera

        # OBJECT DETECTION
        self.object_class = ""
        self.x_difference = 0

        # BUCKET DETECTION
        self.drop_ball_status = 0

        # ORANGE FLARE
        self.orange_search_start = None


        # PUBLISHERS
        self.pub_set_point = self.create_publisher(SetPoint, "set_point", 10)
        self.pub_pid = self.create_publisher(MultiPID, "pid", 10)
        self.pub_status = self.create_publisher(String, "status", 10)


        # SUBSCRIBER
        self.create_subscription(
            ObjectDetection,
            'object_detection',
            self.object_detection_callback,
            10
        )

        self.create_subscription(
            ObjectDifference,
            'object_difference',
            self.object_difference_callback,
            10
        )

        self.create_subscription(
            Float32,
            'drop_ball_msg',
            self.drop_ball_callback,
            10
        )

        self.pub_pid.publish(self.multi_pid)

        # main loop
        self.create_timer(0.1, self.loop)

    # TIME UTILS
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.now() - self.state_start_time

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.now()
        self.get_logger().info(f"STATE -> {new_state}")

    # PUBLISH STATUS
    def publish_status(self, status_str):
        msg      = String()
        msg.data = status_str
        self.pub_status.publish(msg)

    # OBJECT DETECTION
    def object_detection_callback(self, msg):

        if len(msg.bounding_boxes) == 0:
            self.object_class = ""
            return

        self.object_class = msg.bounding_boxes[0].class_name

    # OBJECT DIFFERENCE
    def object_difference_callback(self, msg):
        self.x_difference = msg.x_difference
        self.is_target    = msg.is_target

    # DROP BALL
    def drop_ball_callback(self, msg):
        self.drop_ball_status = msg.data

    # MAIN LOOP
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


    # MISSION LOGIC
    def start_auv(self):

        # STABILIZE
        if self.state == "STABILIZE":

            self.pub_set_point.publish(self.set_point)

            self.publish_status("dpr_ssy")

            if self.elapsed() > 2:
                self.change_state("SEARCH_ORANGE_FLARE")


        # SEARCH FLARE (SCAN)
        # elif self.state == "SEARCH_ORANGE_FLARE":

        #     if self.object_class == "orange_flare":
        #         self.publish_status("camera")

        #         if self.flare_lock_start is None:
        #             self.flare_lock_start = self.now()

        #         lock_time = self.now() - self.flare_lock_start

        #         if lock_time > .9:
        #             self.get_logger().info("FLARE LOCKED")
        #             self.change_state("DODGE_FLARE")

        #     else:
        #         self.flare_lock_start = None

        #         if self.elapsed() > 1:
        #             self.publish_status("all")

        elif self.state == "SEARCH_ORANGE_FLARE":
            if self.orange_search_start is None:
                self.orange_search_start = self.now()
            
            search_time = self.now() - self.orange_search_start

            if search_time > 6.0 and not self.flare_orange_detected_once:
                self.get_logger().info("ORANGE FLARE NOT FOUND 6s → SKIP")
                self.orange_search_start = None
                self.change_state("DODGE_ORANGE_FLARE")
                return

            if self.object_class == "orange_flare":
                # Cek is_target once
                if self.is_target and not self.flare_orange_is_target_once:
                    self.flare_orange_is_target_once = True
                    self.get_logger().info("ORANGE FLARE IS_TARGET ONCE → DPR_SSY")

                if self.flare_orange_is_target_once:
                    # Sudah pernah is_target → diam
                    self.flare_orange_detected_once = True

                    if self.flare_lock_start is None:
                        self.flare_lock_start = self.now()

                    self.publish_status("dpr_ssy")

                    lock_time = self.now() - self.flare_lock_start
                    if lock_time > 1:
                        self.get_logger().info("ORANGE FLARE LOCKED 3s → DODGE")
                        self.flare_orange_is_target_once = False
                        self.flare_orange_detected_once  = False
                        self.flare_lock_start = None
                        self.change_state("DODGE_ORANGE_FLARE")
                else:
                    # Terlihat tapi belum is_target → tetap maju centering
                    self.publish_status("camera")

            else:
                if self.flare_orange_is_target_once:
                    # Pernah is_target → tetap diam meski flare hilang
                    self.publish_status("dpr_ssy")
                else:
                    # Belum pernah is_target → cari
                    if self.elapsed() <= 2:
                        self.publish_status("all")
                    else:
                        self.set_point.yaw = self.base_yaw
                        self.pub_set_point.publish(self.set_point)
                        self.publish_status("all")

        # DODGE FLARE
        elif self.state == "DODGE_ORANGE_FLARE":
            if not hasattr(self, 'no_orange_seen') or self.no_orange_seen is None:
                self.no_orange_seen = self.now()
                
            self.status_dodge_flare = "sway_right_forward"
            self.publish_status("sway_right_forward")
            # if self.x_difference >= 100:
            #     self.status_dodge_flare = "camera_sway_forward_left"
            #     self.publish_status("camera_sway_forward_left")
            # elif self.x_difference < 100:
            #     self.status_dodge_flare = "camera_sway_right_forward"
            #     self.publish_status("camera_sway_right_forward")
            # else:
            #     self.status_dodge_flare = "camera_sway_forward_left"
            #     self.publish_status("camera_sway_forward_left")

            if self.object_class == "orange_flare":
                self.last_orange_flare_seen = self.now()
                self.flare_orange_detected_once = True

            else:
                if self.flare_orange_detected_once:
                    lost_time = self.now() - self.last_orange_flare_seen
                    if lost_time > 4:
                        self.get_logger().info("ORANGE FLARE DODGED")
                        self.change_state("BALANCE_GATE")
                else:
                    lost_time_no_orange_seen = self.now() - self.no_orange_seen
                    if lost_time_no_orange_seen > 4:
                        self.get_logger().info("ORANGE FLARE DODGED")
                        self.change_state("BALANCE_GATE")

        elif self.state == "BALANCE_GATE":
            self.set_point.depth = 0.8
            self.pub_set_point.publish(self.set_point)

            if self.object_class == "gate":
                self.publish_status("camera_sway")
                if -10 <= self.x_difference <= 10:
                    self.get_logger().info("GATE DETECTED")
                    self.change_state("GO_GATE")
            else:
                # self.publish_status("all")
                if self.status_dodge_flare == "sway_right_forward":
                    self.publish_status("sway_right")
                else:
                    self.publish_status("sway_left")

        # SEARCH GATE
        elif self.state == "GO_GATE":
            if not hasattr(self, 'no_gate_seen') or self.no_gate_seen is None:
                self.no_gate_seen = self.now()

            if self.object_class == "gate":
                self.publish_status("camera")

                self.get_logger().info("GATE LOCKED")
                self.last_gate_seen = self.now()
                self.gate_detected_once = True

            else:
                self.publish_status("all")

                if self.gate_detected_once:

                    lost_time = self.now() - self.last_gate_seen

                    if lost_time > 10:
                        self.get_logger().info("GATE PASSED")
                        self.change_state("SEARCH_BUCKET")
                else:
                    lost_time_gate = self.now() - self.no_gate_seen
                    if lost_time_gate > 5:
                        self.get_logger().info("ORANGE FLARE DODGED")
                        self.change_state("BALANCE_GATE")

        # SEARCH BUCKET
        elif self.state == "SEARCH_BUCKET":
            self.set_point.depth = 0.0
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
                    # self.publish_status("all_slow")
                    if self.status_dodge_flare == "sway_right_forward":
                        self.publish_status("sway_left")
                    else:
                        self.publish_status("sway_right")
        
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

        # SURFACE
        elif self.state == "SURFACE":
            self.set_point.depth = -1.3
            self.pub_set_point.publish(self.set_point)

            if not hasattr(self, 'surface') or self.surface_time is None:
                self.surface_time = self.now()

            lost_time = self.now() - self.surface_time

            if lost_time > 1 :
                self.publish_status("stop")
            else :
                self.publish_status("dpr_ssy")


def main(args=None):

    rclpy.init(args=args)

    node = Guidance()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()