#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import numpy as np
import threading
import matplotlib.animation as animation

from std_msgs.msg import String, Float32, Int8, Int16
from geometry_msgs.msg import Pose, Point, Quaternion
from auv_interfaces.msg import SetPoint

# ================= GLOBAL =================
robot_x, robot_y = 0, 0
robot_theta = 90
robot_speed = 0
initial_yaw = 0
boost = 350
robot_theta_display = 90
status = ""
recieve_set_point = False
flag = 0

lock = threading.Lock()

speed_map = {
    ("all", 500): 0.085,
    ("all", 350): 0.075,
    ("backward", 350): 0.0788,
    ("camera", 350): 0.0408,
    ("camera", 0): 0.028,
    ("last_slow", 350): 0.0408,
    ("last_slow", 0): 0.032,
    ("sway_right", 0): 0.0485,
    ("sway_left", 0): 0.0503,
    ("sway_left_forward", 0): 0.048,
    ("sway_right_forward", 0): 0.048
}

def calculate_speed(status, boost):
    return speed_map.get((status, boost), 0)


# ================= NODE =================
class MapNode(Node):
    def __init__(self):
        super().__init__('robot_visualizer')

        # Publisher
        self.pose_pub = self.create_publisher(Pose, '/robot_pose', 10)
        self.zone_pub = self.create_publisher(Int8, 'Zone', 10)

        # Subscriber
        self.create_subscription(String, 'Status', self.status_callback, 10)
        self.create_subscription(Float32, '/yaw_data', self.yaw_callback, 10)
        self.create_subscription(SetPoint, 'SetPoint', self.setpoint_callback, 10)
        self.create_subscription(Float32, 'Boost', self.boost_callback, 10)
        self.create_subscription(Int16, 'Flag', self.flag_callback, 10)

        # Timer (publish pose)
        self.create_timer(0.1, self.publish_pose)

    # ================= CALLBACK =================
    def boost_callback(self, msg):
        global boost
        boost = msg.data

    def setpoint_callback(self, msg):
        global initial_yaw, recieve_set_point
        if not recieve_set_point:
            initial_yaw = msg.yaw
            recieve_set_point = True

    def yaw_callback(self, msg):
        global robot_theta
        with lock:
            normalized_yaw = (msg.data - initial_yaw + 90 + 180) % 360 - 180
            robot_theta = np.radians(normalized_yaw)

    def status_callback(self, msg):
        global status
        status = msg.data.strip().lower()

    def flag_callback(self, msg):
        global flag
        flag = msg.data
        self.get_logger().info(f"Received flag: {flag}")

    # ================= PUBLISH =================
    def publish_pose(self):
        global robot_x, robot_y

        with lock:
            pose_msg = Pose()
            pose_msg.position = Point(x=robot_x, y=robot_y, z=0.0)
            pose_msg.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=np.sin(robot_theta / 2),
                w=np.cos(robot_theta / 2)
            )
            self.pose_pub.publish(pose_msg)


# ================= ROBOT MOTION =================
def move_robot(node):
    global robot_x, robot_y, robot_theta, robot_theta_display

    rate = node.create_rate(10)

    while rclpy.ok():
        with lock:
            speed = calculate_speed(status, boost)

            if status == "stop":
                robot_x = 0
                robot_y = 0
            elif status == "all":
                robot_x += speed * np.cos(robot_theta)
                robot_y += speed * np.sin(robot_theta)
            elif status == "backward":
                robot_x -= speed * np.cos(robot_theta)
                robot_y -= speed * np.sin(robot_theta)
            elif status == "yaw_left":
                robot_theta += np.radians(10)
            elif status == "yaw_right":
                robot_theta -= np.radians(10)

            node.get_logger().info(
                f"Move: {status} | Pos: ({robot_x:.2f}, {robot_y:.2f})"
            )

        rate.sleep()


# ================= PLOT =================
fig, ax = plt.subplots(figsize=(8, 8))
trail_points = []

def setup_plot():
    ax.clear()
    ax.set_xlim(-14, 14)
    ax.set_ylim(0, 25)
    ax.grid(True)

def update_plot(frame):
    setup_plot()

    with lock:
        trail_points.append((robot_x, robot_y))
        if len(trail_points) > 100:
            trail_points.pop(0)

        if len(trail_points) > 1:
            x, y = zip(*trail_points)
            ax.plot(x, y)

        triangle = np.array([[0.3, 0], [-0.3, -0.2], [-0.3, 0.2]])
        R = np.array([
            [np.cos(robot_theta), -np.sin(robot_theta)],
            [np.sin(robot_theta), np.cos(robot_theta)]
        ])
        tri = np.dot(triangle, R.T) + np.array([robot_x, robot_y])
        ax.fill(tri[:, 0], tri[:, 1])


# ================= MAIN =================
def main():
    rclpy.init()
    node = MapNode()

    # Thread ROS2
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Thread movement
    move_thread = threading.Thread(target=move_robot, args=(node,), daemon=True)
    move_thread.start()

    # Plot
    ani = animation.FuncAnimation(fig, update_plot, interval=100)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()