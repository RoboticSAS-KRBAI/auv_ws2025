#!/usr/bin/env python3

import sys
import threading
from PyQt5 import QtWidgets
from .gui_guidance import Ui_MainWindow  # hasil dari pyuic5

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from auv_interfaces.msg import MultiPID, SetPoint


class GuidanceGUI(Node):
    def __init__(self, ui):
        super().__init__('gui_guidance')
        self.ui = ui

        # Subscriptions
        self.sub_pid = self.create_subscription(MultiPID, 'pid', self.pid_callback, 10)
        self.sub_setpoint = self.create_subscription(SetPoint, 'set_point', self.setpoint_callback, 10)
        self.sub_status = self.create_subscription(String, 'status', self.status_callback, 10)

    def pid_callback(self, msg):
        # self.ui.lblYaw.setText(f"Yaw KP: {msg.pid_yaw.kp:.2f}")
        self.ui.lblYaw.setText(f"Yaw KP: {msg.pid_yaw.kp:.2f}" if msg.pid_yaw.kp is not None else "Yaw KP: None")
        self.ui.lblPitch.setText(f"Pitch KP: {msg.pid_pitch.kp:.2f}")
        self.ui.lblRoll.setText(f"Roll KP: {msg.pid_roll.kp:.2f}")
        self.ui.lblDepth.setText(f"Depth KP: {msg.pid_depth.kp:.2f}")

    def setpoint_callback(self, msg):
        self.ui.lblSetpoint.setText(f"Yaw: {msg.yaw:.2f}, Depth: {msg.depth:.2f}")

    def status_callback(self, msg):
        self.ui.lblStatus.setText(f"Status: {msg.data}")


def ros_spin(node):
    """Jalankan ROS2 di thread terpisah supaya GUI tidak nge-freeze"""
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    # Inisialisasi ROS dan GUI
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()

    # Buat node dan jalankan di thread terpisah
    node = GuidanceGUI(ui)
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Jalankan event loop GUI
    sys.exit(app.exec_())



if __name__ == '__main__':
    main()
