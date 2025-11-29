#!/usr/bin/env python3

import sys
import threading
from PyQt5 import QtWidgets, QtCore
from .AUV_GUI import Ui_MainWindow  # hasil dari pyuic5

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import MultiPID, SetPoint, Sensor, PID, MultiPID, SetPoint, Actuator
from std_msgs.msg import String, Float32

# class GuiSignals(QtCore.QObject):
#     sensor_update = QtCore.pyqtSignal(object)

class GuidanceGUI(Node):
    def __init__(self, ui):
        super().__init__('gui_guidance')
        self.ui = ui
        # self.signals = GuiSignals()
        
        # self.signals.sensor_update.connect(self.update_sensor_gui)

        # Subscriptions
        # self.sub_pid = self.create_subscription(MultiPID, 'pid', self.pid_callback, 10)
        self.sub_setpoint = self.create_subscription(SetPoint, 'set_point_msg', self.setpoint_callback, 10)
        self.sub_status = self.create_subscription(String, 'status_msg', self.status_callback, 10)
        self.sub_status_setpoint = self.create_subscription(String, 'status', self.status_setpoint_callback, 10)
        self.sub_sensor = self.create_subscription(Sensor, 'sensor_msg', self.sensor_callback, 10)
        self.sub_actuator = self.create_subscription(Actuator, 'actuator_pwm', self.actuator_callback, 10)

        # Publisher
        self.pub_multi_pid = self.create_publisher(MultiPID, 'pid', 10)
        self.pub_set_point = self.create_publisher(SetPoint, 'set_point', 10)
        self.pub_status = self.create_publisher(String, 'status', 10)
        self.pub_boost = self.create_publisher(Float32, 'boost', 10)

        self.ui.pushButton.clicked.connect(self.publish_values)

        self.get_logger().info("GUI ROS2 Node Started with Publishers")
    
    def publish_values(self):

        try:
            yaw = float(self.ui.setYaw.text())
            pitch = float(self.ui.setPitch.text())
            roll = float(self.ui.setRoll.text())
            depth = float(self.ui.setDepth.text())
        except:
            print("ERROR: Input tidak valid")
            return

        # -------- PID values (bisa kamu ubah) ----------
        pid_yaw = PID()
        pid_yaw.kp = 10.0
        pid_yaw.ki = 0.0
        pid_yaw.kd = 0.0

        pid_pitch = PID()
        pid_pitch.kp = 4000.0
        pid_pitch.ki = 0.0
        pid_pitch.kd = 0.0

        pid_roll = PID()
        pid_roll.kp = 500.0
        pid_roll.ki = 0.0
        pid_roll.kd = 0.0

        pid_depth = PID()
        pid_depth.kp = 3000.0
        pid_depth.ki = 0.0
        pid_depth.kd = 0.0

        multi_pid_msg = MultiPID()
        multi_pid_msg.pid_yaw = pid_yaw
        multi_pid_msg.pid_pitch = pid_pitch
        multi_pid_msg.pid_roll = pid_roll
        multi_pid_msg.pid_depth = pid_depth

        # -------- SetPoint message ----------
        set_point = SetPoint()
        set_point.yaw = yaw
        set_point.pitch = pitch
        set_point.roll = roll
        set_point.depth = depth

        # -------- Status ----------
        status = String()
        status.data = "yaw"

        # -------- Boost ----------
        boost = Float32()
        boost.data = 0.0

        # -------- Publish ----------
        self.pub_status.publish(status)
        self.pub_multi_pid.publish(multi_pid_msg)
        self.pub_set_point.publish(set_point)
        self.pub_boost.publish(boost)

        print("====== PUBLISH SUCCESS ======")
        print("Yaw:", yaw)
        print("Pitch:", pitch)
        print("Roll:", roll)
        print("Depth:", depth)
        print("================================")

    # def pid_callback(self, msg):
        # self.ui.lblYaw.setText(f"Yaw KP: {msg.pid_yaw.kp:.2f}")
        # self.ui.lblYaw.setText(f"Yaw KP: {msg.pid_yaw.kp:.2f}" if msg.pid_yaw.kp is not None else "Yaw KP: None")
        # self.ui.lblPitch.setText(f"Pitch KP: {msg.pid_pitch.kp:.2f}")
        # self.ui.lblRoll.setText(f"Roll KP: {msg.pid_roll.kp:.2f}")
        # self.ui.lblDepth.setText(f"Depth KP: {msg.pid_depth.kp:.2f}")
        # self.ui.yawValue.setText(f"{msg.pid_yaw.kp:.2f}")
        # self.ui.pitchValue.setText(f"{msg.pid_pitch.kp:.2f}")
        # self.ui.rollValue.setText(f"{msg.pid_roll.kp:.2f}")
        # self.ui.depthValue.setText(f"{msg.pid_depth.kp:.2f}")

    def status_callback(self, msg):
        self.ui.Status.setText(msg.data)
    
    def status_setpoint_callback(self, msg):
        self.ui.statusSetPoint.setText(msg.data)

    def sensor_callback(self,msg):
        self.ui.Yaw.setText(f"{msg.yaw:.0f}°")
        self.ui.Depth.setText(f"{msg.depth:.2f}")
        self.ui.Pitch.setText(f"{msg.pitch:.2f}")
        self.ui.Roll.setText(f"{msg.roll:.2f}")

    def setpoint_callback(self, msg):
        # self.ui.lblSetpoint.setText(f"Yaw: {msg.yaw:.2f}, Depth: {msg.depth:.2f}")
        self.ui.yawSetPoint.setText(f"{msg.yaw:.2f}°")
        self.ui.depthSetPoint.setText(f"{msg.depth:.2f}")
        self.ui.pitchSetPoint.setText(f"{msg.pitch:.2f}")
        self.ui.rollSetPoint.setText(f"{msg.roll:.2f}")

    
    # def update_sensor_gui(self, msg):
    #     self.ui.lblYawValue.setText(f"{msg.yaw:.2f}")
    #     self.ui.lblPitchValue.setText(f"{msg.pitch:.2f}")
    #     self.ui.lblRollValue.setText(f"{msg.roll:.2f}")
    #     self.ui.lblDepthValue.setText(f"{msg.depth:.2f}")

    def actuator_callback(self, msg):
        self.ui.Thruster1.setText(f"{msg.thruster_1:.2f}")
        self.ui.Thruster2.setText(f"{msg.thruster_2:.2f}")
        self.ui.Thruster3.setText(f"{msg.thruster_3:.2f}")
        self.ui.Thruster4.setText(f"{msg.thruster_4:.2f}")
        self.ui.Thruster5.setText(f"{msg.thruster_5:.2f}")
        self.ui.Thruster6.setText(f"{msg.thruster_6:.2f}")
        self.ui.Thruster7.setText(f"{msg.thruster_7:.2f}")
        self.ui.Thruster8.setText(f"{msg.thruster_8:.2f}")
        self.ui.Thruster9.setText(f"{msg.thruster_9:.2f}")
        self.ui.Thruster10.setText(f"{msg.thruster_10:.2f}")


def ros_spin(node):
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