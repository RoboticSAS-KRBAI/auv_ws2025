#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from auv_interfaces.msg import ObjectDifference, Sensor


class YawConversionTest(Node):

    def __init__(self):
        super().__init__('yaw_conversion_test')

        # ===== PARAMETER =====
        # Gunakan hasil tuning kamu (bisa diubah-ubah)
        self.K = 0.12   # deg per pixel (kontrol-friendly, bukan full 0.125)

        # ===== DATA =====
        self.x_difference = 0.0
        self.current_yaw  = 0.0

        # ===== SUBSCRIBER =====
        self.create_subscription(
            ObjectDifference,
            'object_difference',
            self.object_callback,
            10
        )

        self.create_subscription(
            Sensor,
            'sensor',
            self.sensor_callback,
            10
        )

        # ===== TIMER =====
        self.create_timer(0.1, self.loop)  # 10 Hz

    # =========================
    # CALLBACK
    # =========================
    def object_callback(self, msg):
        self.x_difference = msg.x_difference

    def sensor_callback(self, msg):
        self.current_yaw = msg.yaw

    # =========================
    # MAIN LOOP
    # =========================
    def loop(self):
        # ===== KONVERSI =====
        yaw_difference = self.K * self.x_difference

        # ===== PREDIKSI YAW =====
        predicted_yaw = self.current_yaw + yaw_difference

        # normalisasi 0–360
        predicted_yaw = predicted_yaw % 360

        # ===== OUTPUT =====
        self.get_logger().info(
            f"\nx_diff: {self.x_difference:.2f} | \n"
            f"yaw_diff: {yaw_difference:.2f} | \n"
            f"yaw_pred: {predicted_yaw:.2f} | \n"
            f"yaw_now: {self.current_yaw:.2f}"
        )


# =========================
# MAIN
# =========================
def main(args=None):
    rclpy.init(args=args)
    node = YawConversionTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()