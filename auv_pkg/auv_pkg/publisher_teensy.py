#!/usr/bin/env python3
import rclpy
from auv_interfaces.msg import PID, MultiPID, SetPoint
from std_msgs.msg import String, Float32

from rclpy.node import Node
import time

class PubTeensy(Node):  #inherit dari class Node nya rclpy (from rclpy.node import Node)

    def __init__(self):  #constructor
        super().__init__("pub_teensy")
        self.get_logger().info("Starting Publisher Teensy Node!!!")

        pub_multi_pid = self.create_publisher(MultiPID, 'pid', 10)
        pub_set_point = self.create_publisher(SetPoint, 'set_point', 10)
        pub_status = self.create_publisher(String, 'status', 10)
        pub_boost = self.create_publisher(Float32, 'boost', 10)
        
        time.sleep(1)

        pid_yaw = PID()
        pid_yaw.kp = 5.0  #7.0 # Proportional constant for yaw
        pid_yaw.ki = 0.0  # Integral constant for yaw
        pid_yaw.kd = 0.3   # Derivative constant for yaw

        pid_pitch = PID()
        pid_pitch.kp = 200.0 #4500.0 #2200 #1700 #1375  # Proportional constant for pitch
        pid_pitch.ki = 0.0   # Integral constant for pitch
        pid_pitch.kd = 1.0    # Derivative constant for pitch

        pid_roll = PID()
        pid_roll.kp = 60.0 #300.0 #500.0   # Proportional constant for roll #50 stabil setelah diubah dari 500
        pid_roll.ki = 0.0     # Integral constant for roll
        pid_roll.kd = 0.51     # Derivative constant for roll

        pid_depth = PID()
        pid_depth.kp = 1300.0 #4000.0 #5000 #1700.0  # Proportional constant for depth
        pid_depth.ki = 0.0     # Integral constant for depth
        pid_depth.kd = 215.0 #500.0     # Derivative constant for depth

        pid_camera = PID()
        pid_camera.kp = 0.5 #1.0      # Proportional constant for camera
        pid_camera.ki = 0.0     # Integral constant for camera
        pid_camera.kd = 0.0     # Derivative constant for camera

        # Create MultiPID message and add multiple PID sets
        multi_pid_msg = MultiPID()
        multi_pid_msg.pid_yaw = pid_yaw
        multi_pid_msg.pid_pitch = pid_pitch
        multi_pid_msg.pid_roll = pid_roll
        multi_pid_msg.pid_depth = pid_depth
        multi_pid_msg.pid_camera = pid_camera
        
        # SetPoint message data
        set_point = SetPoint()
        set_point.yaw = 268.0
        set_point.pitch = 0.00
        set_point.roll = 0.0
        set_point.depth = -0.2 #0.01

        # Status message data
        status = String()
        status.data = "yaw"
        boost = Float32()
        boost.data = 0.0

        self.get_logger().info("-------------SEND DATA-------------")
        print("Status", status)
        print("MultiPID", multi_pid_msg)
        print("Set Point", set_point)
        print("Boost = ", boost)

        pub_status.publish(status)
        pub_multi_pid.publish(multi_pid_msg)
        pub_set_point.publish(set_point)
        pub_boost.publish(boost)
        self.get_logger().info("-------------SUCCESSFUL-------------")
        

def main(args=None):
    rclpy.init(args=args)

    node_pubTeensy = PubTeensy()
    rclpy.spin(node_pubTeensy)
    rclpy.shutdown()


if __name__ == '__main__':
    main()