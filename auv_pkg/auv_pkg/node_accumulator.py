# #!/usr/bin/env python3
# import rclpy
# from auv_interfaces.msg import ObjectDetection, ObjectDifference
# from std_msgs.msg import Int8, Int16, String

# from rclpy.node import Node
# import time

# class SubAccumulator(Node):
#     def __init__(self):
#         super().__init__('accumulator_subscriber')
#         self.object_detection = ObjectDetection()
#         self.object_difference = ObjectDifference()
#         self.object_difference.object_type = "None"
#         self.object_difference.x_difference = 0

#         self.pub_object_difference = self.create_publisher(ObjectDifference, 'object_difference', 10)

#         self.sub_accumulator_objdif = self.create_subscription(
#             ObjectDetection,
#             'object_detection',
#             self.object_detection_callback,
#             10
#         )

#         time.sleep(1)

#     def object_detection_callback(self, data):
#         self.get_logger().info("Received object detection message with " + str(len(data.bounding_boxes)))

#         # Center x-coordinate of the frame
#         frame_center_x = 640 // 2

#         self.object_difference.object_type = "None"
#         self.object_difference.x_difference = 0

        
#         for bbox in data.bounding_boxes:
#             # self.get_logger().info("Class: %s, Probability: %.2f, Coordinates: (%d, %d), (%d, %d)",
#             #             bbox.class_name, bbox.probability, bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max)
            
#             # Calculate the center x-coordinate of the bounding box
#             center_x = (bbox.x_min + bbox.x_max) // 2
            
#             # Calculate the pixel difference between the center of the bounding box and the center of the frame
#             x_difference = center_x - frame_center_x

#             if bbox.class_name == "orange_flare":
#                 self.object_difference.object_type = bbox.class_name
#                 self.object_difference.x_difference = x_difference
#             # if bbox.class_name == "blue_flare":
#             #     self.object_difference.object_type = bbox.class_name
#             #     self.object_difference.x_difference = x_difference
#             if bbox.class_name == "red_flare":
#                 self.object_difference.object_type = bbox.class_name
#                 self.object_difference.x_difference = x_difference
#             if bbox.class_name == "yellow_flare":
#                 self.object_difference.object_type = bbox.class_name
#                 self.object_difference.x_difference = x_difference
#             if bbox.class_name == "Gate":
#                 self.object_difference.object_type = bbox.class_name
#                 self.object_difference.x_difference = x_difference
#             # if bbox.class_name == "Bucket":
#             #     self.object_difference.object_type = bbox.class_name
#             #     self.object_difference.x_difference = x_difference
                
#             # self.get_logger().info("Center x-coordinate of bounding box: %d", center_x)
#             # self.get_logger().info("Pixel difference between bounding box center and frame center: %d", x_difference)
        
#         self.pub_object_difference.publish(self.object_difference)


# def main(args=None):
#     rclpy.init(args=args)
#     node = SubAccumulator()
#     rclpy.spin(node)
#     # node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
import rclpy
from auv_interfaces.msg import ObjectDetection, ObjectDifference
from std_msgs.msg import Int8, Int16, String

from rclpy.node import Node
import time

class SubAccumulator(Node):
    def __init__(self):
        super().__init__('accumulator_subscriber')
        self.object_detection = ObjectDetection()
        self.object_difference = ObjectDifference()
        self.object_difference.object_type = "None"
        self.object_difference.x_difference = 0

        self.pub_object_difference = self.create_publisher(ObjectDifference, 'object_difference', 10)

        self.sub_accumulator_objdif = self.create_subscription(
            ObjectDetection,
            'object_detection',
            self.object_detection_callback,
            10
        )

    def object_detection_callback(self, data):
        self.get_logger().info("Received object detection message with " + str(len(data.bounding_boxes)))

        # Center x-coordinate of the frame
        frame_center_x = 640 // 2

        self.object_difference.object_type = "None"
        self.object_difference.x_difference = 0

        # VALID_CLASSES = ["orange_flare", "yellow_flare", "red_flare", "blue_flare", "gate"]
        # PRIORITY = {"orange_flare": 0, "yellow_flare": 1, "red_flare": 2, "blue_flare": 3, "gate": 5}

        VALID_CLASSES = ["orange_flare", "gate"]
        PRIORITY = {"orange_flare": 0, "gate": 1}

        best_bbox = None
        best_priority = float('inf')

        for bbox in data.bounding_boxes:
            for valid_class in VALID_CLASSES:
                p = PRIORITY[bbox.class_name]
                if p < best_priority:
                    best_priority = p
                    best_bbox = bbox
        
        if best_bbox is not None:
            self.object_difference.object_type = best_bbox.class_name
            bbox_center_x = (best_bbox.x_min + best_bbox.x_max) // 2
            self.object_difference.x_difference = bbox_center_x - frame_center_x
        
        self.pub_object_difference.publish(self.object_difference)


def main(args=None):
    rclpy.init(args=args)
    node = SubAccumulator()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()