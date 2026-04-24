#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import String, Float32, Int8 , Int16
from geometry_msgs.msg import Pose, Point, Quaternion
from auv_interfaces.msg import SetPoint
import threading
import matplotlib.animation as animation
import time

# Initialize robot position and orientation
robot_x, robot_y = 0, 0
robot_theta = 90
robot_speed = 0
initial_yaw = 0
boost = 350
robot_theta_display = 90
status = ""
recieve_set_point = False
recieve_first_flag_3 = False

'''
Status: All, Boost: 500 | Actual Speed: 7.4m/10s = 0.74m/s (Qualification)
Status: All, Boost: 350 | Actual Speed: 6.2m/10s = 0.62m/s
Status: Camera, Boost: 350 | Actual Speed: 4.6m/10s = 0.46m/s
Status: Camera, Boost: 0 | Actual Speed: 3.2m/10s = 0.32m/s
Status: Sway Right | Actual Speed: 2.4m/5s = 0.48m/s
Status: Sway Left | Actual Speed: 2.4m/5s = 0.48m/s
'''

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

# Plot
fig, ax = plt.subplots(figsize=(8, 8))
lock = threading.Lock()

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')

        self.pose_pub = self.create_publisher(Pose, '/robot_pose', 10)
        self.zone_pub = self.create_publisher(Int8, 'Zone', 10)

        self.create_subscription(String, 'Status', self.status_callback, 10)
        self.create_subscription(Float32, '/yaw_data', self.yaw_callback, 10)
        self.create_subscription(SetPoint, 'SetPoint', self.setPoint_callback, 10)
        self.create_subscription(Float32, 'Boost', self.boost_callback, 10)
        self.create_subscription(Int16, 'Flag', self.flag_callback, 10)

    def boost_callback(self, msg):
        global boost
        boost = msg.data

    def setPoint_callback(self, msg):
        global initial_yaw, recieve_set_point
        if not recieve_set_point:
            initial_yaw = msg.yaw
            print(initial_yaw)
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
        global robot_y, recieve_first_flag_3
        if msg.data == 3 and not recieve_first_flag_3:
            robot_y = 14
            recieve_first_flag_3 = True

    def publish_pose_loop(self):
        global robot_x, robot_y
        while rclpy.ok():
            with lock:
                pose_msg = Pose()
                pose_msg.position = Point(robot_x, robot_y, 0.0)
                pose_msg.orientation = Quaternion(
                    0, 0,
                    np.sin(robot_theta / 2),
                    np.cos(robot_theta / 2)
                )
                self.pose_pub.publish(pose_msg)

                if (24 <= robot_y) or (robot_x <= -13) or (13 <= robot_x):
                    self.zone_pub.publish(Int8(data=5))
                elif 18 <= robot_y <= 25 and 0 <= robot_x <= 14:
                    self.zone_pub.publish(Int8(data=1))
                elif 18 <= robot_y <= 25 and -14 <= robot_x <= 0:
                    self.zone_pub.publish(Int8(data=2))
                elif 12 <= robot_y <= 18 and -14 <= robot_x <= 0:
                    self.zone_pub.publish(Int8(data=3))
                elif 12 <= robot_y <= 18 and 0 <= robot_x <= 14:
                    self.zone_pub.publish(Int8(data=4))
                else:
                    self.zone_pub.publish(Int8(data=0))

            time.sleep(0.1)

def move_robot(node):
    global robot_x, robot_y, robot_theta, robot_theta_display
    while rclpy.ok():
        with lock:
            robot_speed = calculate_speed(status, boost)

            if status == "stop":
                robot_x = 0
                robot_y = 0
            elif status == "all":
                robot_x += robot_speed * np.cos(robot_theta)
                robot_y += robot_speed * np.sin(robot_theta)
            elif status == "backward":
                robot_x -= robot_speed * np.cos(robot_theta)
                robot_y -= robot_speed * np.sin(robot_theta)

            node.get_logger().info(
                f"Speed: {robot_speed} | Pos: ({robot_x:.2f},{robot_y:.2f})"
            )

        time.sleep(0.1)

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
        if len(trail_points) > 1:
            x, y = zip(*trail_points)
            ax.plot(x, y)

def main():
    rclpy.init()
    node = RobotVisualizer()

    threading.Thread(target=node.publish_pose_loop, daemon=True).start()
    threading.Thread(target=move_robot, args=(node,), daemon=True).start()

    ani = animation.FuncAnimation(fig, update_plot, interval=100)

    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()