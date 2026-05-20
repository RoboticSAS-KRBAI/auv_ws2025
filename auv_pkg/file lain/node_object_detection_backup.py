#!/usr/bin/env python3
import cv2
import time
import os
import sys
import torch
from torchvision import transforms
from PIL import Image as PILImage

from ultralytics import YOLO
from auv_interfaces.msg import BoundingBox, ObjectDetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

# ===== FUnIE-GAN =====
sys.path.append('/home/techsas/FUnIE-GAN/PyTorch')
from nets import funiegan

# ================= CONFIG =================
IMGSZ = 640
CAM_ID = '/home/techsas/auv_ws/src/auv_pkg/auv_pkg/vid/yellow_flare_telkom_30apr.mp4'
WARMUP_FRAMES = 20
CONF_THRES = 0.5
# ==========================================


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('node_object_detection')

        self.obj_det_pub = self.create_publisher(ObjectDetection, 'object_detection', 10)
        self.pub_image = self.create_publisher(Image, '/camera/main_cam', 10)

        # ===== Load TensorRT Engine =====
        self.model = YOLO(
            '/home/techsas/auv_ws/src/auv_pkg/pt/telkom_kolam_cewe_new_best.engine',
            task='detect'
        )

        # ===== Load FUnIE-GAN =====
        self.funie_model = funiegan.GeneratorFunieGAN()
        self.funie_model.load_state_dict(torch.load(
            '/home/techsas/FUnIE-GAN/PyTorch/models/funie_generator.pth',
            weights_only=False
        ))
        self.funie_model.cuda().eval()
        self.get_logger().info("✅ FUnIE-GAN loaded")

        self.funie_transform = transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5)),
        ])

        # CvBridge
        self.bridge = CvBridge()

        self.get_logger().info("USING CAMERA ID = {}".format(CAM_ID))

        # Image publish interval
        self.last_image_pub = time.time()
        self.image_pub_interval = 0.1  # 10fps

        # ===== Check CUDA =====
        try:
            self.use_cuda = torch.cuda.is_available()
        except ImportError:
            self.use_cuda = False

        if self.use_cuda:
            self.get_logger().info("✅ CUDA is available. Using GPU acceleration.")
        else:
            self.get_logger().warn("⚠️ CUDA is NOT available. Running on CPU, which may be slow.")

        # ===== Camera =====
        self.cap = cv2.VideoCapture(CAM_ID)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMGSZ)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMGSZ)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError("❌ Cannot open camera")

        # ===== Warmup =====
        self.get_logger().warn("🔥 Warming up TensorRT...")
        for _ in range(WARMUP_FRAMES):
            ret, frame = self.cap.read()
            if ret:
                self.model.predict(frame, imgsz=IMGSZ, device=0, verbose=False)
        self.get_logger().warn("✅ Warmup done")

    def enhance_frame(self, frame):
        """CLAHE enhancement sebagai alternatif FUnIE-GAN"""
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        l = clahe.apply(l)
        lab = cv2.merge((l, a, b))
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def spin_once(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # ===== Enhance frame dulu =====
        frame = self.enhance_frame(frame)

        frame_cx, frame_cy = IMGSZ // 2, 480 // 2
        cv2.circle(frame, (frame_cx, frame_cy), 3, (255, 0, 0), -1)

        # ===== Inference YOLO =====
        t0 = time.time()
        results = self.model.predict(
            frame,
            imgsz=IMGSZ,
            device=0,
            verbose=False
        )
        infer_time = time.time() - t0
        fps = 1.0 / infer_time if infer_time > 0 else 0.0

        # ===== Build ROS Message + Draw =====
        msg = ObjectDetection()

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                center_x_vis = (x1 + x2) // 2
                center_y_vis = (y1 + y2) // 2
                cv2.circle(frame, (center_x_vis, center_y_vis), 3, (0, 0, 255), -1)
                cv2.putText(frame, f'({center_x_vis}, {center_y_vis})',
                            (center_x_vis, center_y_vis + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                bbox = BoundingBox()
                bbox.class_name = label
                bbox.probability = conf
                bbox.x_min = x1
                bbox.y_min = y1
                bbox.x_max = x2
                bbox.y_max = y2
                bbox.total_x = x2 - x1
                msg.bounding_boxes.append(bbox)

        self.obj_det_pub.publish(msg)

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(frame, "FUnIE-GAN ON", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        self.get_logger().info(f"[FPS] {fps:.1f} | Detected {len(msg.bounding_boxes)} objects")

        # Publish image 10fps resize 320x240
        if t0 - self.last_image_pub >= self.image_pub_interval:
            small_frame = cv2.resize(frame, (320, 240))
            img_msg = self.bridge.cv2_to_imgmsg(small_frame, encoding='bgr8')
            self.pub_image.publish(img_msg)
            self.last_image_pub = t0

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()
        del self.model
        del self.funie_model
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        while rclpy.ok():
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import cv2
import time
import os
import sys
import torch
from torchvision import transforms
from PIL import Image as PILImage

from ultralytics import YOLO
from auv_interfaces.msg import BoundingBox, ObjectDetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node

# ===== FUnIE-GAN =====
sys.path.append('/home/techsas/FUnIE-GAN/PyTorch')
from nets import funiegan

# ================= CONFIG =================
IMGSZ = 640
CAM_ID = 4
WARMUP_FRAMES = 20
CONF_THRES = 0.5
# ==========================================


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('node_object_detection')

        qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )


        self.obj_det_pub = self.create_publisher(ObjectDetection, 'object_detection', 10)
        self.pub_image = self.create_publisher(Image, '/camera/main_cam', qos)

        # ===== Load TensorRT Engine =====
        self.model = YOLO(
            '/home/techsas/auv_ws/src/auv_pkg/pt/telkom_1may_best.engine',
            task='detect'
        )

        # ===== Load FUnIE-GAN =====
        self.funie_model = funiegan.GeneratorFunieGAN()
        self.funie_model.load_state_dict(torch.load(
            '/home/techsas/FUnIE-GAN/PyTorch/models/funie_generator.pth',
            weights_only=False
        ))
        self.funie_model.cuda().eval()
        self.get_logger().info("✅ FUnIE-GAN loaded")

        self.funie_transform = transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5)),
        ])

        # CvBridge
        self.bridge = CvBridge()

        self.get_logger().info("USING CAMERA ID = {}".format(CAM_ID))

        # Image publish interval
        self.last_image_pub = time.time()
        self.image_pub_interval = 0.1  # 10fps

        # ===== Check CUDA =====
        try:
            self.use_cuda = torch.cuda.is_available()
        except ImportError:
            self.use_cuda = False

        if self.use_cuda:
            self.get_logger().info("✅ CUDA is available. Using GPU acceleration.")
        else:
            self.get_logger().warn("⚠️ CUDA is NOT available. Running on CPU, which may be slow.")

        # ===== Camera =====
        self.cap = cv2.VideoCapture(CAM_ID)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMGSZ)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMGSZ)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError("❌ Cannot open camera")

        # ===== Warmup =====
        self.get_logger().warn("🔥 Warming up TensorRT...")
        for _ in range(WARMUP_FRAMES):
            ret, frame = self.cap.read()
            if ret:
                self.model.predict(frame, imgsz=IMGSZ, device=0, verbose=False)
        self.get_logger().warn("✅ Warmup done")

    def enhance_frame(self, frame):
        """CLAHE enhancement"""
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        lab = cv2.merge((l, a, b))
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def draw_before_after(self, before, after):
        """Gabungkan frame before dan after side by side dengan label"""
        h, w = before.shape[:2]
        half_w = w // 2

        # resize keduanya ke setengah lebar
        before_half = cv2.resize(before, (half_w, h))
        after_half = cv2.resize(after, (half_w, h))

        # gabung side by side
        combined = cv2.hconcat([before_half, after_half])

        # garis pemisah tengah
        cv2.line(combined, (half_w, 0), (half_w, h), (255, 255, 255), 2)

        # label BEFORE
        cv2.rectangle(combined, (0, 0), (110, 35), (0, 0, 0), -1)
        cv2.putText(combined, "BEFORE", (8, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

        # label AFTER
        cv2.rectangle(combined, (half_w, 0), (half_w + 110, 35), (0, 0, 0), -1)
        cv2.putText(combined, "AFTER", (half_w + 8, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        return combined

    def spin_once(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # simpan frame asli untuk before
        frame_before = frame.copy()

        # ===== Enhance frame =====
        frame_after = self.enhance_frame(frame)

        # YOLO pakai frame yang sudah di-enhance
        frame_cx, frame_cy = IMGSZ // 2, 480 // 2
        cv2.circle(frame_after, (frame_cx, frame_cy), 3, (255, 0, 0), -1)

        # ===== Inference YOLO =====
        t0 = time.time()
        results = self.model.predict(
            frame_after,
            imgsz=IMGSZ,
            device=0,
            verbose=False
        )
        infer_time = time.time() - t0
        fps = 1.0 / infer_time if infer_time > 0 else 0.0

        # ===== Build ROS Message + Draw di frame_after =====
        msg = ObjectDetection()

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                cv2.rectangle(frame_before, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame_before, f"{label} {conf:.2f}", (x1, y1 - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.rectangle(frame_after, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame_after, f"{label} {conf:.2f}", (x1, y1 - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                center_x_vis = (x1 + x2) // 2
                center_y_vis = (y1 + y2) // 2
                cv2.circle(frame_after, (center_x_vis, center_y_vis), 3, (0, 0, 255), -1)
                cv2.putText(frame_after, f'({center_x_vis}, {center_y_vis})',
                            (center_x_vis, center_y_vis + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.circle(frame_before, (center_x_vis, center_y_vis), 3, (0, 0, 255), -1)
                cv2.putText(frame_before, f'({center_x_vis}, {center_y_vis})',
                            (center_x_vis, center_y_vis + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                bbox = BoundingBox()
                bbox.class_name = label
                bbox.probability = conf
                bbox.x_min = x1
                bbox.y_min = y1
                bbox.x_max = x2
                bbox.y_max = y2
                bbox.total_x = x2 - x1
                msg.bounding_boxes.append(bbox)

        self.obj_det_pub.publish(msg)

        # FPS info di frame_after
        cv2.putText(frame_after, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(frame_after, "CLAHE ON", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        self.get_logger().info(f"[FPS] {fps:.1f} | Detected {len(msg.bounding_boxes)} objects")

        # ===== Gabungkan before/after untuk publish =====
        combined = self.draw_before_after(frame_before, frame_after)

        # Publish image 10fps resize 320x240 (before/after combined)
        if t0 - self.last_image_pub >= self.image_pub_interval:
            # publish combined before/after, resize supaya tidak terlalu besar
            small_frame = cv2.resize(combined, (640, 240))
            img_msg = self.bridge.cv2_to_imgmsg(small_frame, encoding='bgr8')
            self.pub_image.publish(img_msg)
            self.last_image_pub = t0

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()
        del self.model
        del self.funie_model
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        while rclpy.ok():
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import cv2
import time
import os
import sys
import torch
from torchvision import transforms
from PIL import Image as PILImage

from ultralytics import YOLO
from auv_interfaces.msg import BoundingBox, ObjectDetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node

# ===== FUnIE-GAN =====
sys.path.append('/home/techsas/FUnIE-GAN/PyTorch')
from nets import funiegan

# ================= CONFIG =================
IMGSZ = 640
CAM_ID = 4
WARMUP_FRAMES = 20
CONF_THRES = 0.5
# ==========================================


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('node_object_detection')

        qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )


        self.obj_det_pub = self.create_publisher(ObjectDetection, 'object_detection', 10)
        self.pub_image = self.create_publisher(Image, '/camera/main_cam', qos)

        # ===== Load TensorRT Engine =====
        self.model = YOLO(
            '/home/techsas/auv_ws/src/auv_pkg/pt/telkom_1may_best.engine',
            task='detect'
        )

        # ===== Load FUnIE-GAN =====
        self.funie_model = funiegan.GeneratorFunieGAN()
        self.funie_model.load_state_dict(torch.load(
            '/home/techsas/FUnIE-GAN/PyTorch/models/funie_generator.pth',
            weights_only=False
        ))
        self.funie_model.cuda().eval()
        self.get_logger().info("✅ FUnIE-GAN loaded")

        self.funie_transform = transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5)),
        ])

        # CvBridge
        self.bridge = CvBridge()

        self.get_logger().info("USING CAMERA ID = {}".format(CAM_ID))

        # Image publish interval
        self.last_image_pub = time.time()
        self.image_pub_interval = 0.1  # 10fps

        # ===== Check CUDA =====
        try:
            self.use_cuda = torch.cuda.is_available()
        except ImportError:
            self.use_cuda = False

        if self.use_cuda:
            self.get_logger().info("✅ CUDA is available. Using GPU acceleration.")
        else:
            self.get_logger().warn("⚠️ CUDA is NOT available. Running on CPU, which may be slow.")

        # ===== Camera =====
        self.cap = cv2.VideoCapture(CAM_ID)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMGSZ)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMGSZ)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError("❌ Cannot open camera")

        # ===== Warmup =====
        self.get_logger().warn("🔥 Warming up TensorRT...")
        for _ in range(WARMUP_FRAMES):
            ret, frame = self.cap.read()
            if ret:
                self.model.predict(frame, imgsz=IMGSZ, device=0, verbose=False)
        self.get_logger().warn("✅ Warmup done")

    def enhance_frame(self, frame):
        """CLAHE enhancement"""
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        lab = cv2.merge((l, a, b))
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def draw_before_after(self, before, after):
        """Gabungkan frame before dan after side by side dengan label"""
        h, w = before.shape[:2]
        half_w = w // 2

        # resize keduanya ke setengah lebar
        before_half = cv2.resize(before, (half_w, h))
        after_half = cv2.resize(after, (half_w, h))

        # gabung side by side
        combined = cv2.hconcat([before_half, after_half])

        # garis pemisah tengah
        cv2.line(combined, (half_w, 0), (half_w, h), (255, 255, 255), 2)

        # label BEFORE
        cv2.rectangle(combined, (0, 0), (110, 35), (0, 0, 0), -1)
        cv2.putText(combined, "BEFORE", (8, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

        # label AFTER
        cv2.rectangle(combined, (half_w, 0), (half_w + 110, 35), (0, 0, 0), -1)
        cv2.putText(combined, "AFTER", (half_w + 8, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        return combined

    def spin_once(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # simpan frame asli untuk before
        frame_before = frame.copy()

        # ===== Enhance frame =====
        frame_after = self.enhance_frame(frame)

        # YOLO pakai frame yang sudah di-enhance
        frame_cx, frame_cy = IMGSZ // 2, 480 // 2
        cv2.circle(frame_after, (frame_cx, frame_cy), 3, (255, 0, 0), -1)

        # ===== Inference YOLO =====
        t0 = time.time()
        results = self.model.predict(
            frame_after,
            imgsz=IMGSZ,
            device=0,
            verbose=False
        )
        infer_time = time.time() - t0
        fps = 1.0 / infer_time if infer_time > 0 else 0.0

        # ===== Build ROS Message + Draw di frame_after =====
        msg = ObjectDetection()

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                cv2.rectangle(frame_before, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame_before, f"{label} {conf:.2f}", (x1, y1 - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.rectangle(frame_after, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame_after, f"{label} {conf:.2f}", (x1, y1 - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                center_x_vis = (x1 + x2) // 2
                center_y_vis = (y1 + y2) // 2
                cv2.circle(frame_after, (center_x_vis, center_y_vis), 3, (0, 0, 255), -1)
                cv2.putText(frame_after, f'({center_x_vis}, {center_y_vis})',
                            (center_x_vis, center_y_vis + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.circle(frame_before, (center_x_vis, center_y_vis), 3, (0, 0, 255), -1)
                cv2.putText(frame_before, f'({center_x_vis}, {center_y_vis})',
                            (center_x_vis, center_y_vis + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                bbox = BoundingBox()
                bbox.class_name = label
                bbox.probability = conf
                bbox.x_min = x1
                bbox.y_min = y1
                bbox.x_max = x2
                bbox.y_max = y2
                bbox.total_x = x2 - x1
                msg.bounding_boxes.append(bbox)

        self.obj_det_pub.publish(msg)

        # FPS info di frame_after
        cv2.putText(frame_after, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(frame_after, "CLAHE ON", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        self.get_logger().info(f"[FPS] {fps:.1f} | Detected {len(msg.bounding_boxes)} objects")

        # ===== Gabungkan before/after untuk publish =====
        combined = self.draw_before_after(frame_before, frame_after)

        # Publish image 10fps resize 320x240 (before/after combined)
        if t0 - self.last_image_pub >= self.image_pub_interval:
            # publish combined before/after, resize supaya tidak terlalu besar
            small_frame = cv2.resize(combined, (640, 240))
            img_msg = self.bridge.cv2_to_imgmsg(small_frame, encoding='bgr8')
            self.pub_image.publish(img_msg)
            self.last_image_pub = t0

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()
        del self.model
        del self.funie_model
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        while rclpy.ok():
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()