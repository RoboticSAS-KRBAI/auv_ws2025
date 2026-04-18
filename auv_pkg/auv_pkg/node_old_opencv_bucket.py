#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int8, Int16, String


class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')

        self.get_logger().info("Node OpenCV has been started")

        # Open camera
        self.cap = cv2.VideoCapture(2)  # Using the primary camera (index 0)

        # Set the frame width and height
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.center_x = 0
        self.center_y = 0

        # Set original video frame rate

        # HSV values for color detection
        # lower_hue = 104
        # upper_hue = 179
        # lower_saturation = 84
        # upper_saturation = 255
        # lower_value = 156
        # upper_value = 255

        self.lower_hue = 82
        self.upper_hue = 179
        self.lower_saturation = 167
        self.upper_saturation = 255
        self.lower_value = 114
        self.upper_value = 255

        # State
        self.object_detected = False
        self.drop_bucket = "False"
        self.flag = 0  # ganti

        # Publisher
        self.pub = self.create_publisher(String, '/bucket_detected_python', 10)
        self.pub1 = self.create_publisher(Bool, '/bucket_detected', 10)

        # Subscriber
        self.sub = self.create_subscription(
            Int16,
            'Flag',
            self.callback_flag,
            10
        )

        # Timer loop
        self.timer = self.create_timer(0.03, self.process_frame)

    def detect_color(self, frame, lower_hue, upper_hue, lower_saturation, upper_saturation, lower_value, upper_value):
        # Convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color range in HSV
        lower_bound = np.array([lower_hue, lower_saturation, lower_value])
        upper_bound = np.array([upper_hue, upper_saturation, upper_value])

        # Masking color based on the defined range
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Display mask result
        result = cv2.bitwise_and(frame, frame, mask=mask)

        return result, mask

    def callback_flag(self, data: Int16):
        self.flag = data.data

    def process_frame(self):
        # Read frame from the camera
        ret, frame = self.cap.read()

        if not ret:
            return

        # Detect color based on HSV values
        color_detected_frame, mask = self.detect_color(
            frame,
            self.lower_hue,
            self.upper_hue,
            self.lower_saturation,
            self.upper_saturation,
            self.lower_value,
            self.upper_value
        )

        # Find contours from the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        bounding_box_area = 0

        if contours:
            # Find the largest contour based on area
            largest_contour = max(contours, key=cv2.contourArea)
            bounding_box_area = cv2.contourArea(largest_contour)

            # Create a bounding box around the object with the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Calculate the center point of the bounding box
            self.center_x = x + w // 2
            self.center_y = y + h // 2
            cv2.circle(frame, (self.center_x, self.center_y), 2, (0, 255, 0), -1)
            cv2.putText(
                frame,
                f"Object Center: ({self.center_x}, {self.center_y}) | Bounding Box: {bounding_box_area}",
                (self.center_x - 20, self.center_y - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2
            )

        # Find the center point of the frame
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2
        cv2.circle(frame, (frame_center_x, frame_center_y), 2, (255, 0, 0), -1)
        cv2.putText(
            frame,
            f"Frame Center: ({frame_center_x}, {frame_center_y})",
            (frame_center_x + 10, frame_center_y + 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2
        )

        if abs(frame_center_x - self.center_x) <= 35 and abs(frame_center_y - self.center_y) <= 35 and bounding_box_area >= 180000 and self.flag == 4:
            self.get_logger().info("Ball Dropping")
            self.object_detected = True
            self.drop_bucket = "True"

        # elif abs(frame_center_x - center_x) <= 35 and abs(frame_center_y - center_y) <= 35 and bounding_box_area >= 100000:
        #     rospy.loginfo("Maju Dikit")
        #     drop_bucket = "Maju"

        # elif frame_center_x < center_x and bounding_box_area >= 150000:
        #     rospy.loginfo("Berada di Kanan")
        #     # objek berada di kanan
        #     drop_bucket = "Kanan"

        # elif frame_center_x > center_x and bounding_box_area >= 150000:
        #     rospy.loginfo("Berada di Kiri")
        #     # objek berada di kiri
        #     drop_bucket = "Kiri"

        else:
            self.object_detected = False

        # Publish
        msg_bool = Bool()
        msg_bool.data = self.object_detected

        msg_string = String()
        msg_string.data = self.drop_bucket

        self.pub1.publish(msg_bool)
        self.pub.publish(msg_string)

        # Display the results
        cv2.imshow('Original Video', frame)
        # cv2.imshow('Color Detection', color_detected_frame)

        # Break the loop by pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ColorDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()