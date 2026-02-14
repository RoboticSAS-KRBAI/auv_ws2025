#!/usr/bin/env python3
import cv2
import time

from ultralytics import YOLO
from auv_interfaces.msg import BoundingBox, ObjectDetection

import rclpy
from rclpy.node import Node

# from pathlib import Path

# from ament_index_python.packages import get_package_share_directory
# model_path = Path(get_package_share_directory('reynard_pkg')) / 'models' / 'SAUVC2_100.pt'


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('node_object_detection')

        self.obj_det_pub = self.create_publisher(ObjectDetection, 'object_detection', 10)

        self.model = YOLO('yolo11n.pt')
        self.use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0

        x = 640
        y = 480
        self.frame_center_x = x // 2
        self.frame_center_y = y // 2

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) #MJPG lebih ringan dari YUYV
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, x)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, y)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            exit()

        self.timer = self.create_timer(0.03, self.timer_callback)  # approx. 30 FPS

    def timer_callback(self):
        start_time = time.time()

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Can't receive frame. Skipping...")
            return

        if self.use_cuda:
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            frame = gpu_frame.download()

        results = self.model.predict(frame, imgsz=320, device=0)
        obj_det_msg = ObjectDetection()

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                # Draw rectangle and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)
                cv2.putText(frame, f'({center_x}, {center_y})', (center_x, center_y + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                bbox_msg = BoundingBox()
                bbox_msg.class_name = label
                bbox_msg.probability = conf
                bbox_msg.x_min = x1
                bbox_msg.y_min = y1
                bbox_msg.x_max = x2
                bbox_msg.y_max = y2
                bbox_msg.total_x = x2 - x1
                obj_det_msg.bounding_boxes.append(bbox_msg)

        # Draw center point of frame
        cv2.circle(frame, (self.frame_center_x, self.frame_center_y), 3, (255, 0, 0), -1)
        cv2.putText(frame, f'({self.frame_center_x}, {self.frame_center_y})',
                    (self.frame_center_x + 10, self.frame_center_y + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        self.obj_det_pub.publish(obj_det_msg)
        cv2.imshow("Camera Feed", frame)

        fps = 1.0 / (time.time() - start_time)
        self.get_logger().info(f"FPS: {fps:.2f}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

