#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from auv_interfaces.msg import ObjectDetection, ObjectDifference
from std_msgs.msg import Int16, String


class Subscriber(Node):
    def __init__(self):
        super().__init__('node_accumulator')

        self.object_detection = ObjectDetection()
        self.object_difference = ObjectDifference()

        self.object_difference.object_type = "None"
        self.object_difference.x_difference = 0
        self.object_difference.is_target = False

        # Publisher
        self.pub_object_difference = self.create_publisher(
            ObjectDifference,
            'object_difference',
            10
        )

        # Subscribers
        self.create_subscription(
            ObjectDetection,
            'object_detection',
            self.object_detection_callback,
            10
        )

        self.create_subscription(
            Int16,
            'Flag',
            self.flag_callback,
            10
        )

        self.create_subscription(
            String,
            'Target_flare',
            self.target_flare_callback,
            10
        )

        self.mode = 1
        self.flag = 0
        self.flare_value = ''

        self.get_logger().info("Node Accumulator Berjalan")

    def flag_callback(self, data: Int16):
        self.flag = data.data

    def target_flare_callback(self, data: String):
        self.flare_value = data.data

    def object_detection_callback(self, data):
        frame_center_x = 640 // 2
        detected_objects = {}

        # Simpan objek
        for bbox in data.bounding_boxes:
            detected_objects[bbox.class_name] = bbox

        priority_list = []

        # === MODE LOGIC ===
        if self.mode == 1:
            if self.flag == 0:
                priority_list = ["Orange_Flare"]
            elif self.flag == 1:
                priority_list = ["Orange_Flare", "Gate"]
            elif self.flag == 2:
                priority_list = ["Gate"]
            elif self.flag == 3:
                if self.flare_value == 'B':
                    self.get_logger().info("HANYA MENERIMA FLARE BIRU")
                    priority_list = ["Blue_Flare"]
                elif self.flare_value == 'R':
                    self.get_logger().info("HANYA MENERIMA FLARE MERAH")
                    priority_list = ["Red_Flare"]
                elif self.flare_value == 'Y':
                    self.get_logger().info("HANYA MENERIMA FLARE KUNING")
                    priority_list = ["Yellow_Flare"]
                elif self.flare_value == 'O':
                    self.get_logger().info("HANYA MENERIMA FLARE ORANGE")
                    priority_list = ["Orange_Flare"]
            elif self.flag == 4:
                self.get_logger().info("HANYA MENERIMA BUCKET BIRU")
                priority_list = ["Blue_Bucket"]

        elif self.mode == 2:
            if self.flag in [0, 1]:
                priority_list = ["Gate"]
            elif self.flag == 3:
                if self.flare_value == 'B':
                    self.get_logger().info("HANYA MENERIMA FLARE BIRU")
                    priority_list = ["Blue_Flare"]
                elif self.flare_value == 'R':
                    self.get_logger().info("HANYA MENERIMA FLARE MERAH")
                    priority_list = ["Red_Flare"]
                elif self.flare_value == 'Y':
                    self.get_logger().info("HANYA MENERIMA FLARE KUNING")
                    priority_list = ["Yellow_Flare"]
                elif self.flare_value == 'O':
                    self.get_logger().info("HANYA MENERIMA FLARE ORANGE")
                    priority_list = ["Orange_Flare"]

        # === PRIORITY SELECTION ===
        detected_object = None
        priority_found = False

        for obj_name in priority_list:
            if obj_name in detected_objects:
                detected_object = detected_objects[obj_name]
                self.object_difference.object_type = obj_name
                self.object_difference.is_target = True
                priority_found = True
                break

        # fallback flare lain
        if not priority_found:
            for obj_name in detected_objects:
                if "Flare" in obj_name:
                    detected_object = detected_objects[obj_name]
                    self.object_difference.object_type = obj_name
                    self.object_difference.is_target = False
                    break

        # === HITUNG OUTPUT ===
        if detected_object:
            self.get_logger().info(
                f"Detected: {self.object_difference.object_type} "
                f"(Target: {self.object_difference.is_target})"
            )

            self.object_difference.x_difference = (
                (detected_object.x_min + detected_object.x_max) // 2
                - frame_center_x
            )

            self.object_difference.bounding_box_size = (
                detected_object.x_max - detected_object.x_min
            )

        else:
            self.object_difference.object_type = "None"
            self.object_difference.x_difference = 0
            self.object_difference.is_target = False
            self.object_difference.bounding_box_size = 0

        self.pub_object_difference.publish(self.object_difference)


def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()