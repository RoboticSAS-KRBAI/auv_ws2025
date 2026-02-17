#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct

PORT = "/dev/ttyACM0"
BAUD = 115200

FMT = "<Ih"   # uint32 t_ms, int16 pwm
PACKET_SIZE = struct.calcsize(FMT)


class DeadbandSerialPublisher(Node):
    def __init__(self):
        super().__init__('deadband_serial_publisher')

        self.pub = self.create_publisher(
            Int32MultiArray,
            'deadband/thruster',
            10
        )

        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        self.timer = self.create_timer(0.01, self.read_serial)  # 100 Hz

        self.get_logger().info("Deadband serial publisher started")

    def read_serial(self):
        data = self.ser.read(PACKET_SIZE)
        if len(data) != PACKET_SIZE:
            return

        t_ms, pwm = struct.unpack(FMT, data)

        msg = Int32MultiArray()
        msg.data = [int(t_ms), int(pwm)]

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DeadbandSerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
