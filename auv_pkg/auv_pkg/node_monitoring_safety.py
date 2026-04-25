import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import String, Bool

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # Serial
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Publisher (ESP32 → ROS2)
        self.pub = self.create_publisher(String, 'sensor_string', 10)

        # Subscriber (ROS2 → ESP32)
        self.sub = self.create_subscription(
            Bool,
            'safety_flag',
            self.bool_callback,
            10
        )

        # Timer untuk baca serial
        self.timer = self.create_timer(0.1, self.read_serial)

        self.get_logger().info("Serial Bridge Node Started")

    # =========================
    # SERIAL → ROS2
    # =========================
    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()

            self.get_logger().info(f"Raw serial: {line}")

            if not line:
                return

            if line.startswith("timestamp"):
                return
            

            data = line.split(',')

            if len(data) != 13:
                self.get_logger().warn(f"Format salah: {len(data)} kolom")
                return

            # Parse sebagian (biar lebih informatif)
            timestamp = int(data[0])
            pressure = float(data[1])
            temp_fusion = float(data[4])
            humidity = float(data[5])

            voltage1 = float(data[6])
            current1 = float(data[7])
            power1 = float(data[8])

            voltage2 = float(data[9])
            current2 = float(data[10])
            power2 = float(data[11])

            hids_valid = int(data[12])

            # Format string (lebih readable)
            formatted = (
                f"t={timestamp} | "
                f"p={pressure:.2f}hPa | "
                f"temp={temp_fusion:.2f}C | "
                f"hum={humidity:.2f}% | "
                f"v1={voltage1:.2f}V i1={current1:.3f}A p1={power1:.2f}W | "
                f"v2={voltage2:.2f}V i2={current2:.3f}A p2={power2:.2f}W | "
                f"hids={'OK' if hids_valid else 'FAIL'}"
            )

            msg = String()
            msg.data = formatted

            self.pub.publish(msg)

            self.get_logger().info(f"[PUB] {formatted}")

        except Exception as e:
            self.get_logger().error(f"Read error: {e}")

    # =========================
    # ROS2 → SERIAL
    # =========================
    def bool_callback(self, msg: Bool):
        try:
            if msg.data:
                self.ser.write(b'1\n')
                self.get_logger().info("[SERIAL] Sent: 1")
            else:
                self.ser.write(b'0\n')
                self.get_logger().info("[SERIAL] Sent: 0")

        except Exception as e:
            self.get_logger().error(f"Write error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()