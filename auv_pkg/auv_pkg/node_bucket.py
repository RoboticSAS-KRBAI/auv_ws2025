#!/usr/bin/env python3
"""
camera_bucket_detection.py

Node ROS2 untuk:
- Kamera deteksi warna (bucket)
- Publish hasil deteksi ke ROS2 topic
- Subscribe safety_flag dari ROS2
- Kirim drop ball ke Teensy via ROS2 topic /drop_ball
"""

import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ================= CONFIG =================
CAM_ID   = 0
IMGSZ_W  = 640
IMGSZ_H  = 480
# ==========================================


class CameraBucketDetectionNode(Node):
    def __init__(self):
        super().__init__('camera_bucket_detection_node')
        self.get_logger().info("Node Camera Bucket Detection has been started")

        # -------------------------------------------------
        # KAMERA
        # -------------------------------------------------
        self.cap = cv2.VideoCapture(CAM_ID)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  IMGSZ_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMGSZ_H)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera id={CAM_ID}")
            self.cap = None
        else:
            w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            f = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f"Camera actual: {w}x{h} @ {f}fps")

        # -------------------------------------------------
        # HSV THRESHOLD
        # -------------------------------------------------
        self.lower_hue        = 82
        self.upper_hue        = 179
        self.lower_saturation = 167
        self.upper_saturation = 255
        self.lower_value      = 114
        self.upper_value      = 255

        # -------------------------------------------------
        # DETECTION STATE
        # -------------------------------------------------
        self.center_x        = 0
        self.center_y        = 0
        self.object_detected = False
        self.drop_bucket     = "False"

        # -------------------------------------------------
        # FPS TRACKING
        # -------------------------------------------------
        self.fps = 0.0
        self.t0  = time.time()

        # -------------------------------------------------
        # CVBRIDGE
        # -------------------------------------------------
        self.bridge = CvBridge()

        # -------------------------------------------------
        # IMAGE PUBLISH INTERVAL (~10fps)
        # -------------------------------------------------
        self.last_image_pub     = time.time()
        self.image_pub_interval = 0.1

        # -------------------------------------------------
        # DROP BALL SEND INTERVAL
        # -------------------------------------------------
        self.last_serial_send     = time.time()
        self.serial_send_interval = 0.1

        # -------------------------------------------------
        # PUBLISHERS
        # -------------------------------------------------
        self.pub_string    = self.create_publisher(String, '/bucket_detected_python', 10)
        self.pub_bool      = self.create_publisher(Bool,   '/bucket_detected',        10)
        self.pub_image     = self.create_publisher(Image,  '/camera/bucket_cam',      10)
        self.pub_drop_ball = self.create_publisher(String, '/drop_ball',              10)

        # -------------------------------------------------
        # SUBSCRIBERS
        # -------------------------------------------------
        self.sub_safety = self.create_subscription(
            Bool,
            'safety_flag',
            self.safety_callback,
            10
        )

        self.get_logger().info("Node Started")

    # =========================================================================
    # COLOR DETECTION
    # =========================================================================
    def detect_color(self, frame):
        """Deteksi warna berdasarkan HSV threshold."""
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower  = np.array([self.lower_hue,  self.lower_saturation,  self.lower_value])
        upper  = np.array([self.upper_hue,  self.upper_saturation,  self.upper_value])
        mask   = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        return result, mask

    # =========================================================================
    # SPIN ONCE
    # =========================================================================
    def spin_once(self):
        if self.cap is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame not received!")
            return

        # Hitung FPS
        infer_time = time.time() - self.t0
        self.fps   = 1.0 / infer_time if infer_time > 0 else 0.0
        self.t0    = time.time()

        now = time.time()

        # Deteksi warna
        _, mask = self.detect_color(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        frame_cx    = frame.shape[1] // 2
        frame_cy    = frame.shape[0] // 2
        THRESHOLD_X = 50
        THRESHOLD_Y = 80

        if contours:
            largest_contour   = max(contours, key=cv2.contourArea)
            bounding_box_area = cv2.contourArea(largest_contour)
            x, y, w, h        = cv2.boundingRect(largest_contour)
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

        # Gambar zona deteksi (kotak merah di tengah)
        cv2.rectangle(
            frame,
            (frame_cx - THRESHOLD_X, frame_cy - THRESHOLD_Y),
            (frame_cx + THRESHOLD_X, frame_cy + THRESHOLD_Y),
            (0, 0, 255), 2
        )
        cv2.circle(frame, (frame_cx, frame_cy), 3, (255, 0, 0), -1)

        # FPS overlay
        cv2.putText(
            frame, f"FPS: {self.fps:.1f}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 255, 0), 2
        )

        # Logika deteksi: objek harus ada contour DAN masuk zona tengah
        in_zone_x = abs(frame_cx - self.center_x) <= THRESHOLD_X
        in_zone_y = abs(frame_cy - self.center_y) <= THRESHOLD_Y

        if contours and in_zone_x and in_zone_y:
            self.object_detected = True
            self.drop_bucket     = "True"
        else:
            self.object_detected = False
            self.drop_bucket     = "False"

        # Kirim drop ball ke Teensy via ROS2 (throttled)
        if now - self.last_serial_send >= self.serial_send_interval:
            self.send_drop_ball(self.object_detected)
            self.last_serial_send = now

        # Publish ke ROS2
        msg_bool      = Bool()
        msg_bool.data = self.object_detected
        self.pub_bool.publish(msg_bool)

        msg_str      = String()
        msg_str.data = self.drop_bucket
        self.pub_string.publish(msg_str)

        self.get_logger().info(
            f"[FPS] {self.fps:.1f} | detected={self.object_detected} | "
            f"center=({self.center_x},{self.center_y})"
        )

        # Publish image (throttled, resize 320x240)
        if now - self.last_image_pub >= self.image_pub_interval:
            small_frame = cv2.resize(frame, (320, 240))
            img_msg     = self.bridge.cv2_to_imgmsg(small_frame, encoding='bgr8')
            self.pub_image.publish(img_msg)
            self.last_image_pub = now

    # =========================================================================
    # PUBLISH DROP BALL → Teensy via ROS2 topic /drop_ball
    # =========================================================================
    def send_drop_ball(self, detected: bool):
        msg      = String()
        msg.data = "putar" if detected else "diam"
        self.pub_drop_ball.publish(msg)

    # =========================================================================
    # SAFETY CALLBACK
    # =========================================================================
    def safety_callback(self, msg: Bool):
        """Callback dari topic safety_flag, forward ke Teensy via ROS2."""
        self.send_drop_ball(msg.data)

    # =========================================================================
    # CLEANUP
    # =========================================================================
    def destroy(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = CameraBucketDetectionNode()

    try:
        while rclpy.ok():
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()