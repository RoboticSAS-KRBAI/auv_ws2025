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

        time.sleep(1)

    def object_detection_callback(self, data):
        self.get_logger().info("Received object detection message with " + str(len(data.bounding_boxes)))

        # Center x-coordinate of the frame
        frame_center_x = 640 // 2

        self.object_difference.object_type = "None"
        self.object_difference.x_difference = 0

        
        for bbox in data.bounding_boxes:
            # self.get_logger().info("Class: %s, Probability: %.2f, Coordinates: (%d, %d), (%d, %d)",
            #             bbox.class_name, bbox.probability, bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max)
            
            # Calculate the center x-coordinate of the bounding box
            center_x = (bbox.x_min + bbox.x_max) // 2
            
            # Calculate the pixel difference between the center of the bounding box and the center of the frame
            x_difference = center_x - frame_center_x

            if bbox.class_name == "Orange_Flare":
                self.object_difference.object_type = bbox.class_name
                self.object_difference.x_difference = x_difference
            if bbox.class_name == "Gate":
                self.object_difference.object_type = bbox.class_name
                self.object_difference.x_difference = x_difference
            if bbox.class_name == "Bucket":
                self.object_difference.object_type = bbox.class_name
                self.object_difference.x_difference = x_difference
                
            # self.get_logger().info("Center x-coordinate of bounding box: %d", center_x)
            # self.get_logger().info("Pixel difference between bounding box center and frame center: %d", x_difference)
        
        self.pub_object_difference.publish(self.object_difference)


def main(args=None):
    rclpy.init(args=args)
    node = SubAccumulator()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

