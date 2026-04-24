#!/usr/bin/env python3

# tambah command buat record pas jalanin file ini atau manual lewat terminal aja?

import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import String, Float32, Int8, Int16
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDifference, Sensor
from geometry_msgs.msg import Pose


class Subscriber(Node):
    def __init__(self):
        super().__init__('node_guidance')

        self.is_start = False
        self.start_time_flag = 0
        self.boot_time = 0
        self.start_time = 0
        self.current_time = 0
        self.elapsed_time = 0
        self.status = "stop"
        self.boost = 350
        self.has_published_value = False
        self.flag = 3
        self.param_delay = 5
        self.param_duration = 0

        self.start_rotation_time = None
        self.elapsed_rotation = 0

        self.set_point = SetPoint()
        self.multi_pid_msg = MultiPID()

        self.set_point.roll = 0
        self.set_point.pitch = 0
        self.set_point.depth = 0.9

        self.depth_surface = 0.13
        self.set_point_yaw_front = 85
        self.set_point.yaw = self.set_point_yaw_front
        self.set_point_yaw_right = self.set_point.yaw - 90
        self.set_point_yaw_left = self.set_point.yaw + 92
        self.set_point_yaw_back = self.set_point.yaw + 180

        self.zone = 0
        self.target_zone = 0
        self.target_x = 0
        self.target_y = 0
        self.robot_x = 0
        self.robot_y = 0
        self.stabilized = False
        self.scenario_zone = [18, 15]
        self.current_zone = 0
        self.done_zone = False

        self.start_to_back = None
        self.elapsed_to_back = 0

        # Publisher
        self.pub_multi_pid = self.create_publisher(MultiPID, "PID", 10)
        self.pub_set_point = self.create_publisher(SetPoint, "SetPoint", 10)
        self.pub_status = self.create_publisher(String, "Status", 10)
        self.pub_boost = self.create_publisher(Float32, "Boost", 10)
        self.pub_flag = self.create_publisher(Int16, "Flag", 10)

        # Subscriber
        self.create_subscription(Pose, '/robot_pose', self.callback_pose, 10)
        self.create_subscription(Int8, 'Zone', self.callback_zone, 10)

        # PID init
        self.pid_yaw = PID()
        self.pid_yaw.Kp = 10.0
        self.pid_yaw.Ki = 0.0
        self.pid_yaw.Kd = 1.0

        self.pid_pitch = PID()
        self.pid_pitch.Kp = 7000.0
        self.pid_pitch.Ki = 0.0
        self.pid_pitch.Kd = 800.0

        self.pid_roll = PID()
        self.pid_roll.Kp = 700.0
        self.pid_roll.Ki = 0.0
        self.pid_roll.Kd = 0.0

        self.pid_depth = PID()
        self.pid_depth.Kp = 3000.0
        self.pid_depth.Ki = 0.0
        self.pid_depth.Kd = 0.0

        self.pid_camera = PID()
        self.pid_camera.Kp = 1.0
        self.pid_camera.Ki = 0.0
        self.pid_camera.Kd = 0.0

        self.multi_pid_msg.pid_yaw = self.pid_yaw
        self.multi_pid_msg.pid_pitch = self.pid_pitch
        self.multi_pid_msg.pid_roll = self.pid_roll
        self.multi_pid_msg.pid_depth = self.pid_depth
        self.multi_pid_msg.pid_camera = self.pid_camera

        # Timer (pengganti loop while)
        self.timer = self.create_timer(0.1, self.loop)

    def loop(self):
        self.start()

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180

    def callback_pose(self, data: Pose):
        self.robot_x = data.position.x
        self.robot_y = data.position.y

    def callback_zone(self, data: Int8):
        self.zone = data.data

    def is_in_range(self, start_time, end_time):
        return (self.boot_time > start_time + self.param_delay and end_time is None) or \
               ((start_time + self.param_delay) < self.boot_time < (end_time + self.param_delay))

    def publish_string(self, pub, text):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def publish_float(self, pub, val):
        msg = Float32()
        msg.data = float(val)
        pub.publish(msg)

    def publish_int(self, pub, val):
        msg = Int16()
        msg.data = int(val)
        pub.publish(msg)

    def correct_position(self):
        self.get_logger().warn("Correct Position")

        tolerance_x = 0.5
        tolerance_y = 0.5

        if abs(self.robot_y - self.target_y) > tolerance_y:
            if self.robot_y < self.target_y:
                self.set_point.yaw = self.set_point_yaw_front
                self.publish_string(self.pub_status, "all")
            else:
                self.set_point.yaw = self.normalize_angle(self.set_point_yaw_back)
                self.publish_string(self.pub_status, "all")

            self.pub_set_point.publish(self.set_point)
            self.start_rotation_time = None
            return

        if abs(self.robot_x - self.target_x) > tolerance_x:
            if self.robot_x < self.target_x:
                self.set_point.yaw = self.normalize_angle(self.set_point_yaw_right)
            else:
                self.set_point.yaw = self.normalize_angle(self.set_point_yaw_left)

            self.pub_set_point.publish(self.set_point)
            self.publish_string(self.pub_status, "all")
            self.start_rotation_time = None
            return

        self.get_logger().info("X dan Y sudah sesuai")
        self.stabilized = True
        self.start_rotation_time = self.now()

    def stabilize_and_advance(self):
        self.elapsed_rotation = self.now() - self.start_rotation_time

        if not self.done_zone:
            if self.elapsed_rotation <= 30:
                if self.target_x <= 0:
                    self.publish_string(self.pub_status, "yaw_right")
                else:
                    self.publish_string(self.pub_status, "yaw_left")
            else:
                self.done_zone = True

        else:
            if self.current_zone < len(self.scenario_zone) - 1:
                self.current_zone += 1
            else:
                self.flag = 4

            self.start_rotation_time = None
            self.stabilized = False
            self.done_zone = False

    def start(self):
        if not self.is_start:
            self.start_time = self.now()
            self.is_start = True

        self.boot_time = self.now() - self.start_time

        if self.boot_time < self.param_delay:
            self.get_logger().info('STARTING...')
            return

        if self.param_duration <= 0 or self.boot_time < self.param_duration:
            self.start_auv()

    def start_auv(self):
        if not self.has_published_value:
            self.pub_multi_pid.publish(self.multi_pid_msg)
            self.pub_set_point.publish(self.set_point)
            self.publish_int(self.pub_flag, self.flag)
            self.publish_float(self.pub_boost, self.boost)
            self.has_published_value = True

        if self.is_in_range(0, 4):
            self.publish_string(self.pub_status, "dpr_ssy")

        elif self.flag == 3:
            self.target_zone = self.scenario_zone[self.current_zone]

            zone_coordinates = {
                18: (7, 17),
                15: (7, 14),
            }

            self.target_x, self.target_y = zone_coordinates[self.target_zone]

            if not self.stabilized:
                self.correct_position()
            else:
                self.stabilize_and_advance()

        elif self.flag == 4:
            self.set_point.depth = self.depth_surface
            self.pub_set_point.publish(self.set_point)
            self.publish_string(self.pub_status, "dpr")


def main():
    rclpy.init()
    node = Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()