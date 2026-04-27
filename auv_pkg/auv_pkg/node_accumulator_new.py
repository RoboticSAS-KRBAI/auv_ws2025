#!/usr/bin/env python3
import rclpy
from auv_interfaces.msg import ObjectDetection, ObjectDifference
from rclpy.node import Node

# Urutan prioritas — index lebih kecil = lebih prioritas
# Tinggal tambah nama class di sini kalau mau tambah object baru
PRIORITY_ORDER = ["orange_flare", "red_flare", "yellow_flare", "blue_flare", "Gate"]

FRAME_W = 640
FRAME_H = 480
FULL_FRAME_THRESHOLD = FRAME_W * FRAME_H * 0.07  # 20% frame = "sudah dekat"


class SubAccumulator(Node):
    def __init__(self):
        super().__init__('accumulator_subscriber')

        self.object_difference = ObjectDifference()
        self.object_difference.object_type = "None"
        self.object_difference.x_difference = 0
        self.object_difference.is_target = False
        self.object_difference.bounding_box_size = 0

        self.pub_object_difference = self.create_publisher(
            ObjectDifference, 'object_difference', 10
        )

        self.create_subscription(
            ObjectDetection,
            'object_detection',
            self.object_detection_callback,
            10
        )

    def object_detection_callback(self, data):
        self.get_logger().info(
            f"Received object_detection: {len(data.bounding_boxes)} bbox(es)"
        )

        # Reset setiap callback
        self.object_difference.object_type = "None"
        self.object_difference.x_difference = 0
        self.object_difference.is_target = False
        self.object_difference.bounding_box_size = 0

        frame_center_x = FRAME_W // 2

        best_bbox = None
        best_index = float('inf')

        for bbox in data.bounding_boxes:
            if bbox.class_name not in PRIORITY_ORDER:
                self.get_logger().warn(f"Unknown class '{bbox.class_name}', skipping.")
                continue

            index = PRIORITY_ORDER.index(bbox.class_name)
            if index < best_index:
                best_index = index
                best_bbox = bbox

        if best_bbox is not None:
            bbox_area = (best_bbox.x_max - best_bbox.x_min) * \
                        (best_bbox.y_max - best_bbox.y_min)
            center_x = (best_bbox.x_min + best_bbox.x_max) // 2

            self.object_difference.object_type = best_bbox.class_name
            self.object_difference.x_difference = center_x - frame_center_x
            self.object_difference.bounding_box_size = bbox_area
            self.object_difference.is_target = bbox_area >= FULL_FRAME_THRESHOLD

            self.get_logger().info(
                f"Best: {best_bbox.class_name} | "
                f"x_diff={self.object_difference.x_difference} | "
                f"area={bbox_area} | "
                f"is_target={self.object_difference.is_target}"
            )

        self.pub_object_difference.publish(self.object_difference)


def main(args=None):
    rclpy.init(args=args)
    node = SubAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()