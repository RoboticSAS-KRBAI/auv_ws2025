#!/usr/bin/env python3
"""
serial_monitoring_safety_and_bucket.py

Node ROS2 untuk:
- Kamera deteksi warna (bucket) — Auto-switch V4L2 untuk kamera / non-V4L2 untuk file video
- Pengaturan FPS / Delay khusus untuk file video agar tidak terlalu cepat
- Monitoring sensor baterai via serial ESP
- Publish hasil ke ROS2 topic
- Kirim drop ball ke Teensy via ROS2 topic /drop_ball
"""

import os
import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ================= CONFIG =================
CAM_ID   = 0
IMGSZ_W  = 640
IMGSZ_H  = 480
TARGET_VIDEO_FPS = 50  # Target kecepatan pemutaran untuk file video (.mp4)
# ==========================================


class SerialBridgeNodeAndColorDetection(Node):
    def __init__(self):
        super().__init__('serial_bridge_and_color_detection_node')
        self.get_logger().info("Node OpenCV has been started")

        # -------------------------------------------------
        # KAMERA — Auto-detect hardware vs file video
        # -------------------------------------------------
        self.is_video_file = not (isinstance(CAM_ID, int) or (isinstance(CAM_ID, str) and CAM_ID.isdigit()))

        if self.is_video_file:
            self.get_logger().info(f"Membuka file video: {CAM_ID}")
            self.cap = cv2.VideoCapture(CAM_ID)
            # Hitung interval waktu per frame berdasarkan target FPS (30 FPS -> ~0.033 detik)
            self.frame_duration = 1.0 / TARGET_VIDEO_FPS
        else:
            self.get_logger().info(f"Membuka kamera fisik ID: {CAM_ID}")
            cam_idx = int(CAM_ID)
            self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  IMGSZ_W)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMGSZ_H)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.frame_duration = 0.0 # Tidak ada delay buatan untuk kamera fisik

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera/video id={CAM_ID}")
            self.cap = None
        else:
            w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            f = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f"Source actual: {w}x{h} @ {f}fps")


        self.last_log_time = 0
        self.log_interval = 0.05  # 20 Hz

        self.last_frame_time = time.time()
        self.camera_hang_threshold = 0.5

        # -------------------------------------------------
        # HSV THRESHOLD
        # -------------------------------------------------
        self.lower_hue        = 76
        self.upper_hue        = 125
        self.lower_saturation = 164
        self.upper_saturation = 255
        self.lower_value      = 0
        self.upper_value      = 255
    
        # -------------------------------------------------
        # DETECTION STATE
        # -------------------------------------------------
        self.center_x        = 0
        self.center_y        = 0
        self.object_detected = False
        self.drop_bucket     = "False"
        self.done_drop_ball     = False
        self.drop_active = False
        self.drop_start_time = 0
        self.drop_duration = 0.2  # detik
        self.current_status = ""  # Tambahan untuk menyimpan status terkini

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
        # SERIAL (ESP)
        # -------------------------------------------------
        serial_port = '/dev/ttyUSB0'
        self.ser    = None

        if os.path.exists(serial_port):
            try:
                self.ser = serial.Serial(serial_port, 115200, timeout=0.1)
                self.get_logger().info(f"ESP Serial connected: {serial_port}")
            except serial.SerialException as e:
                self.get_logger().warn(f"ESP Serial open failed: {e}")
        else:
            self.get_logger().warn(f"ESP port {serial_port} not found, skipping serial")

        self.last_serial_read     = time.time()
        self.serial_interval      = 0.1

        # -------------------------------------------------
        # IMAGE PUBLISH INTERVAL (~10fps)
        # -------------------------------------------------
        self.last_image_pub     = time.time()
        self.image_pub_interval = 0.1

        # -------------------------------------------------
        # PUBLISHERS & SUBSCRIBERS
        # -------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_string    = self.create_publisher(String, '/bucket_detected_python', 10)
        self.pub_bool      = self.create_publisher(Bool,   '/bucket_detected',        10)
        self.pub_sensor    = self.create_publisher(String, 'sensor_string',           10)
        self.pub_image     = self.create_publisher(Image, '/camera/bucket_cam', qos)
        self.pub_drop_ball = self.create_publisher(Float32, '/drop_ball', 10)

        self.sub_status = self.create_subscription(String, 'status_msg', self.status_callback, 10)
        

        self.get_logger().info("Node Started Successfully")

    # =========================================================================
    # SUBSCRIBER CALLBACK
    # =========================================================================
    def status_callback(self, msg):
        self.current_status = msg.data

    # =========================================================================
    # SERIAL → ROS2
    # =========================================================================
    def read_serial(self):
        if self.ser is None:
            return

        try:
            if self.ser.in_waiting <= 0:
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
                f"t={timestamp} | p={pressure:.2f}hPa | temp={temp_fusion:.2f}C | "
                f"v1={voltage1:.2f}V | v2={voltage2:.2f}V | hids={'OK' if hids_valid else 'FAIL'}"
            )

            msg      = String()
            msg.data = formatted
            self.pub_sensor.publish(msg)

        except Exception as e:
            pass

    # =========================================================================
    # COLOR DETECTION
    # =========================================================================
    def detect_color(self, frame):
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

        # Catat waktu awal pemrosesan frame untuk menghitung durasi delay dinamis
        start_time = time.time()

        self.cap.grab()
        ret, frame = self.cap.retrieve()

        now_check = time.time()

        # Handle loop video seandainya habis
        if not ret:
            if self.is_video_file:
                self.get_logger().info("Video selesai, me-restart dari awal...")
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            else:
                self.get_logger().warn("Frame not received from camera!")
                return

        # WATCHDOG CAMERA HANG
        if now_check - self.last_frame_time > self.camera_hang_threshold:
            if not self.is_video_file:  # Jangan warning jika ini file video yang sengaja didelay
                self.get_logger().warn("⚠️ Camera stall detected")
        self.last_frame_time = now_check

        # Hitung Real FPS
        infer_time = time.time() - self.t0
        self.fps   = 1.0 / infer_time if infer_time > 0 else 0.0
        self.t0    = time.time()

        now = time.time()

        # Baca serial ESP (throttled)
        if now - self.last_serial_read >= self.serial_interval:
            self.read_serial()
            self.last_serial_read = now

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

        # Gambar zona deteksi
        cv2.rectangle(frame, (frame_cx - THRESHOLD_X, frame_cy - THRESHOLD_Y), (frame_cx + THRESHOLD_X, frame_cy + THRESHOLD_Y), (0, 0, 255), 2)
        cv2.circle(frame, (frame_cx, frame_cy), 3, (255, 0, 0), -1)

        # FPS overlay
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # Logika deteksi
        in_zone_x = abs(frame_cx - self.center_x) <= THRESHOLD_X
        in_zone_y = abs(frame_cy - self.center_y) <= THRESHOLD_Y

        if contours and in_zone_x and in_zone_y:
            self.object_detected = True
            self.drop_bucket     = "True"
        else:
            self.object_detected = False
            self.drop_bucket     = "False"

        # Logika Drop Ball (Hanya aktif jika warna terdeteksi DAN status adalah "camera_slow")
        if self.object_detected and self.current_status == "camera_slow" and not self.drop_active and not self.done_drop_ball :
            self.drop_active = True
            self.drop_start_time = now
            self.send_drop_ball(True)
            self.get_logger().info("DROP START")

        if self.drop_active:
            if now - self.drop_start_time >= self.drop_duration:
                self.send_drop_ball(False)
                self.drop_active = False
                self.done_drop_ball = True
                self.get_logger().info("DROP STOP")

        # Publish data deteksi
        msg_bool      = Bool()
        msg_bool.data = self.object_detected
        self.pub_bool.publish(msg_bool)

        msg_str      = String()
        msg_str.data = self.drop_bucket
        self.pub_string.publish(msg_str)

        # Publish image (throttled, resize 320x240)
        if now - self.last_image_pub >= self.image_pub_interval:
            small_frame = cv2.resize(frame, (320, 240))
            img_msg     = self.bridge.cv2_to_imgmsg(small_frame, encoding='bgr8')
            self.pub_image.publish(img_msg)
            self.last_image_pub = now

        # ---- LOGIKA DELAY DINAMIS KHUSUS FILE VIDEO ----
        if self.is_video_file:
            elapsed = time.time() - start_time
            sleep_time = self.frame_duration - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # =========================================================================
    # PUBLISH DROP BALL
    # =========================================================================
    def send_drop_ball(self, detected: bool):
        msg = Float32()
        msg.data = 1.0 if detected else 0.0
        self.pub_drop_ball.publish(msg)

    def destroy(self):
        if self.cap is not None:
            self.cap.release()
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNodeAndColorDetection()

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