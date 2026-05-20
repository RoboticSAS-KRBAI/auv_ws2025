#!/usr/bin/env python3
import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import String, Bool


class SerialBridgeNodeAndColorDetection(Node):
    def __init__(self):
        super().__init__('serial_bridge_and_color_detection_node')
        self.get_logger().info("Node OpenCV has been started")

        # Camera
        self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")


        # Verifikasi
        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        f = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera actual: {w}x{h} @ {f}fps")

        # HSV threshold
        self.lower_hue = 82
        self.upper_hue = 179
        self.lower_saturation = 167
        self.upper_saturation = 255
        self.lower_value = 114
        self.upper_value = 255

        # Detection state
        self.center_x = 0
        self.center_y = 0
        self.object_detected = False
        self.drop_bucket = "False"

        # FPS tracking
        self.fps = 0.0
        self.t0 = time.time()

        # Serial
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Serial read timer
        self.last_serial_read = time.time()
        self.serial_interval = 0.1  # baca serial tiap 100ms

        # Publishers
        self.pub_string = self.create_publisher(String, '/bucket_detected_python', 10)
        self.pub_bool = self.create_publisher(Bool, '/bucket_detected', 10)
        self.pub_sensor = self.create_publisher(String, 'sensor_string', 10)

        # Subscriber
        self.sub_safety = self.create_subscription(
            Bool,
            'safety_flag',
            self.safety_callback,
            10
        )

        self.get_logger().info("Node Started")

    # =========================
    # SERIAL → ROS2 (dipanggil manual di loop)
    # =========================
    def read_serial(self):
        try:
            if self.ser.in_waiting == 0:
                return

            line = self.ser.readline().decode(errors='ignore').strip()
            if not line or line.startswith("timestamp"):
                return

            data = line.split(',')
            if len(data) != 13:
                return

            timestamp   = int(data[0])
            pressure    = float(data[1])
            temp_fusion = float(data[4])
            humidity    = float(data[5])
            voltage1    = float(data[6])
            current1    = float(data[7])
            power1      = float(data[8])
            voltage2    = float(data[9])
            current2    = float(data[10])
            power2      = float(data[11])
            hids_valid  = int(data[12])

            formatted = (
                f"t={timestamp} | "
                f"p={pressure:.2f}hPa | "
                f"temp={temp_fusion:.2f}C | "
                f"hum={humidity:.2f}% | "
                f"v1={voltage1:.3f}V i1={current1:.3f}A p1={power1:.3f}W | "
                f"v2={voltage2:.3f}V i2={current2:.3f}A p2={power2:.3f}W | "
                f"hids={'OK' if hids_valid else 'FAIL'}"
            )

            msg = String()
            msg.data = formatted
            self.pub_sensor.publish(msg)
            self.get_logger().info(f"[SENSOR] {formatted}")

        except Exception as e:
            self.get_logger().error(f"Read serial error: {e}")

    # =========================
    # COLOR DETECTION
    # =========================
    def detect_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([self.lower_hue, self.lower_saturation, self.lower_value])
        upper = np.array([self.upper_hue, self.upper_saturation, self.upper_value])
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        return result, mask

    # =========================
    # SPIN ONCE (seperti pola YOLO)
    # =========================
    def spin_once(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Hitung FPS
        infer_time = time.time() - self.t0
        self.fps = 1.0 / infer_time if infer_time > 0 else 0.0
        self.t0 = time.time()

        # Baca serial setiap interval
        now = time.time()
        if now - self.last_serial_read >= self.serial_interval:
            self.read_serial()
            self.last_serial_read = now

        _, mask = self.detect_color(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        frame_cx = frame.shape[1] // 2
        frame_cy = frame.shape[0] // 2

        THRESHOLD_X = 50
        THRESHOLD_Y = 80

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            bounding_box_area = cv2.contourArea(largest_contour)

            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            self.center_x = x + w // 2
            self.center_y = y + h // 2
            cv2.circle(frame, (self.center_x, self.center_y), 4, (0, 255, 0), -1)
            cv2.putText(
                frame,
                f"Center: ({self.center_x}, {self.center_y}) | Area: {bounding_box_area:.0f}",
                (self.center_x - 20, self.center_y - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
            )

        # Zona deteksi (kotak merah)
        cv2.rectangle(
            frame,
            (frame_cx - THRESHOLD_X, frame_cy - THRESHOLD_Y),
            (frame_cx + THRESHOLD_X, frame_cy + THRESHOLD_Y),
            (0, 0, 255), 2
        )
        cv2.circle(frame, (frame_cx, frame_cy), 3, (255, 0, 0), -1)

        # Logika deteksi
        if abs(frame_cx - self.center_x) <= THRESHOLD_X and abs(frame_cy - self.center_y) <= THRESHOLD_Y:
            self.get_logger().info("Ball Dropping")
            self.object_detected = True
            self.drop_bucket = "True"
        else:
            self.object_detected = False
            self.drop_bucket = "False"

        # Kirim ke ESP32 langsung (tidak delay)
        self.send_to_esp(self.object_detected)

        # Publish ROS2
        msg_bool = Bool()
        msg_bool.data = self.object_detected
        self.pub_bool.publish(msg_bool)

        msg_str = String()
        msg_str.data = self.drop_bucket
        self.pub_string.publish(msg_str)

        # FPS overlay
        cv2.putText(
            frame, f"FPS: {self.fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 255, 0), 2
        )

        cv2.imshow('Original Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt

    # =========================
    # ROS2 → SERIAL
    # =========================
    def send_to_esp(self, detected: bool):
        try:
            cmd = "1\n" if detected else "0\n"
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f"Write serial error: {e}")

    def safety_callback(self, msg: Bool):
        self.send_to_esp(msg.data)

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNodeAndColorDetection()

    try:
        while rclpy.ok():
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.0)  # proses callback ROS2 tanpa blocking
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()