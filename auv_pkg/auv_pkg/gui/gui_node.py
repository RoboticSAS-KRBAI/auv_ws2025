#!/usr/bin/env python3

import sys
import threading
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QPixmap, QTransform, QPainter
from .AUV_GUI import Ui_MainWindow  # hasil dari pyuic5

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import MultiPID, SetPoint, Sensor, PID, MultiPID, SetPoint, Actuator
from std_msgs.msg import String, Float32

# class GuiSignals(QtCore.QObject):
#     sensor_update = QtCore.pyqtS[" was not closedPylanceignal(object)

class GuidanceGUI(Node):
    def __init__(self, ui):
        super().__init__('gui_guidance')
        self.ui = ui
        # self.signals = GuiSignals()
        
        # self.signals.sensor_update.connect(self.update_sensor_gui)

        #yaw_dot

        self.dot_pixmap = QPixmap("/home/reynard/Documents/clone_auv_ws/auv_pkg/auv_pkg/gui/dot1.png").scaled(
            390, 390,
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation
        )

        self.ui.labeldot.setScaledContents(False)

        self.ui.labeldot.setPixmap(self.dot_pixmap)

        self.rotate_timer = QtCore.QTimer()
        # self.rotate_timer.timeout.connect(self.updateYaw)
        self.rotate_timer.start(50)


        # Subscriptions
        # self.sub_pid = self.create_subscription(MultiPID, 'pid', self.pid_callback, 10)
        self.sub_setpoint = self.create_subscription(SetPoint, 'set_point', self.setpoint_callback, 10)
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
            statusText = self.ui.comboBoxStatus.currentText()
            yaw = float(self.ui.setYaw.text())
            depth = float(self.ui.setDepth.text())
            pitch = float(self.ui.setPitch.text())
            roll = float(self.ui.setRoll.text())
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
        status.data = statusText


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

        self.ui.setYaw.setText("")
    
    def updateYaw(self, yaw):
        angle = yaw

        transform = QTransform().rotate(angle)
        rotated = self.dot_pixmap.transformed(transform, QtCore.Qt.SmoothTransformation)

        self.ui.labeldot.setPixmap(rotated)


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
        # self.ui.graphicsViewdot.rotate(round(msg.yaw, 0))
        yaw_angle = round(msg.yaw, 0)
        self.updateYaw(yaw_angle)

    
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