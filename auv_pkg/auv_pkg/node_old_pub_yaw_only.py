#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from auv_interfaces.msg import Sensor  # Pastikan sesuai dengan definisi pesan di ROS

class SensorRelay(Node):
    def __init__(self):
        super().__init__("sensor_relay")

        # Publisher
        self.yaw_pub = self.create_publisher(Float32, "/yaw_data", 10)

        # Subscriber
        self.create_subscription(
            Sensor,
            "/sensor_msg",
            self.sensor_callback,
            10
        )

    # Callback untuk menangani pesan yang diterima
    def sensor_callback(self, msg):
        # self.get_logger().info(f"Roll: {msg.roll}, Pitch: {msg.pitch}, Yaw: {msg.yaw}, Sway: {msg.sway}")
        # self.get_logger().info(f"Depth: {msg.depth}, BusVoltage: {msg.busvoltage}, ShuntVoltage: {msg.shuntvoltage}")
        # self.get_logger().info(f"LoadVoltage: {msg.loadvoltage}, Current: {msg.current_mA} mA, Power: {msg.power_mW} mW")
        # self.get_logger().info(f"Temp: {msg.temperature}°C, Humidity: {msg.humidity}%, Pressure Abs: {msg.pressure_abs} hPa")
        # self.get_logger().info(f"Pressure Rel: {msg.pressure_relative} hPa, Altitude Delta: {msg.altitude_delta} m")
        # self.get_logger().info("-------------------------------------------------")

        # Publish hanya nilai yaw
        yaw_msg = Float32()
        yaw_msg.data = msg.yaw
        self.yaw_pub.publish(yaw_msg)

        # self.get_logger().info(f"Published Yaw: {msg.yaw}")


def main(args=None):
    rclpy.init(args=args)

    node = SensorRelay()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()