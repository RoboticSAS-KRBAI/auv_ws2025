#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDetection, ObjectDifference


class Guidance(Node):

    def __init__(self):
        super().__init__('guidance')

        # TIME
        self.start_time = None
        self.state_start_time = None

        # STATE
        self.state = "INIT"

        # OBJECT DETECTION
        self.object_class = ""

        # PASSED OBJECTS
        self.passed_objects = [5]

        # SCAN
        self.base_yaw = 262.0
        self.scan_angle = 30.0
        self.scan_left = True

        # FLARE LOCK AND LOST
        self.flare_lock_start = None
        self.flare_orange_detected_once = False
        self.last_orange_flare_seen = 0

        # GATE TRACKING
        self.gate_detected_once = False
        self.last_gate_seen = 0

        # BLUE FLARE
        self.color_flare_hit_time = None 
        self.backward_from_flare = None

        # SETPOINT
        self.set_point = SetPoint()
        self.set_point.roll = 0.0
        self.set_point.pitch = 0.0
        self.set_point.yaw = self.base_yaw
        self.set_point.depth = -0.13

        # PID
        self.multi_pid = MultiPID()

        pid_yaw = PID()
        pid_yaw.kp = 4.5
        pid_yaw.kd = 0.3

        pid_pitch = PID()
        pid_pitch.kp = 9.0
        pid_pitch.kd = 1.7

        pid_roll = PID()
        pid_roll.kp = 1.5
        pid_roll.kd = 0.3

        pid_depth = PID()
        pid_depth.kp = 1350.0
        pid_depth.kd = 215.0

        pid_camera = PID()
        pid_camera.kp = 0.11

        self.multi_pid.pid_yaw = pid_yaw
        self.multi_pid.pid_pitch = pid_pitch
        self.multi_pid.pid_roll = pid_roll
        self.multi_pid.pid_depth = pid_depth
        self.multi_pid.pid_camera = pid_camera

        # OBJECT DETECTION
        self.object_class = ""
        self.x_difference = 0


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

    # MAIN LOOP
    def loop(self):

        if self.start_time is None:
            self.start_time = self.now()
            self.change_state("STABILIZE")

        self.start_auv()

    # MISSION LOGIC
    def start_auv(self):

        status = String()

        # STABILIZE
        if self.state == "STABILIZE":

            self.pub_set_point.publish(self.set_point)

            status.data = "dpr_ssy"
            self.pub_status.publish(status)

            if self.elapsed() > 2:
                self.change_state("SEARCH_ORANGE_FLARE")


        # SEARCH FLARE (SCAN)
        elif self.state == "SEARCH_ORANGE_FLARE":

            if self.object_class == "orange_flare" or self.object_class == "orange_flare":
                status.data = "camera"

                if self.flare_lock_start is None:
                    self.flare_lock_start = self.now()

                lock_time = self.now() - self.flare_lock_start

                if lock_time > .9:
                    self.get_logger().info("FLARE LOCKED")
                    self.change_state("DODGE_FLARE")

            else:
                self.flare_lock_start = None

                if self.elapsed() > 1:
                    status.data = "all"
                    self.pub_status.publish(status)

        # DODGE FLARE
        elif self.state == "DODGE_FLARE":
            if self.x_difference >= 100:
                status.data = "camera_sway_forward_left"
                self.pub_status.publish(status)
            elif self.x_difference < 100:
                status.data = "camera_sway_forward_right"
                self.pub_status.publish(status)

            if self.object_class == "orange_flare":
                self.last_orange_flare_seen = self.now()
                self.flare_orange_detected_once = True

            else:
                if self.flare_orange_detected_once:
                
                    lost_time = self.now() - self.last_orange_flare_seen

                    if lost_time > 3:
                        self.get_logger().info("FLARE DODGED")
                        self.change_state("TABRAK_BLUE")

        elif self.state == "TABRAK_BLUE":
            self.publish_status("dpr_ssy")
            self.pub_status.publish(status)

            self.set_point.yaw = self.base_yaw - self.scan_angle
            self.pub_set_point.publish(self.set_point)

            if self.object_class == "blue_flare":
                self.publish_status("camera")

                if self.is_target:
                    if self.color_flare_hit_time is None:
                        self.color_flare_hit_time = self.now()
                        self.get_logger().info(
                            f"FULL FRAME: {self.current_color_flare}, dwell 1.5s"
                        )

                    if self.now() - self.color_flare_hit_time > 1.5:
                        self._substate_start   = self.now()
                        self.change_state = "BACK"
                        self.get_logger().info("DWELL DONE → BACK")
                    else:
                        self.color_flare_hit_time = None  # belum penuh, reset timer
        
        elif self.state == "BACK":
            self.publish_status("backward")

            self.backward_from_flare = self.now()

            if self.now() - self.backward_from_flare > 1.5:
                status.data = "dpr_ssy"
                self.pub_status.publish(status)
                self.pub_set_point.publish(self.set_point)
                self.change_state("BALANCE_GATE")
                


        elif self.state == "BALANCE_GATE":
            self.set_point.depth = 0.2
            self.pub_set_point.publish(self.set_point)

            if self.object_class == "gate":
                status.data = "camera_sway"
                self.pub_status.publish(status)
                if -10 <= self.x_difference <= 10:
                    self.get_logger().info("GATE DETECTED")
                    self.change_state("GO_GATE")

        # SEARCH GATE
        elif self.state == "GO_GATE":

            if self.object_class == "gate":
                status.data = "camera"
                self.pub_status.publish(status)

                self.get_logger().info("GATE LOCKED")
                self.last_gate_seen = self.now()
                self.gate_detected_once = True

            else:

                if self.gate_detected_once:

                    lost_time = self.now() - self.last_gate_seen

                    if lost_time > 4:
                        self.get_logger().info("GATE PASSED")
                        self.change_state("SURFACE")


        # SURFACE
        elif self.state == "SURFACE":
            self.set_point.depth = -0.51
            self.pub_set_point.publish(self.set_point)

            status.data = "dpr_ssy"
            self.pub_status.publish(status)


def main(args=None):

    rclpy.init(args=args)

    node = Guidance()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()