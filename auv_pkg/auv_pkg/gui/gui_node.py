#!/usr/bin/env python3
import math
import sys
import threading
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QPixmap, QTransform, QPainter
from .AUV_GUI import Ui_MainWindow  # hasil dari pyuic5
from .ModelAUV import ROV3DWidget
# from PyQt5.QtOpenGL import QGLWidget

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import MultiPID, SetPoint, Sensor, PID, MultiPID, SetPoint, Actuator
from std_msgs.msg import String, Float32

# from .camera_node import CameraThread

class GuidanceGUI(Node):
    def __init__(self, ui):
        super().__init__('gui_guidance')
        self.ui = ui

        # 1. Buat GLWidget baru
        self.opengl_widget = ROV3DWidget(self.ui.openGLWidget.parent())

        # 2. Pastikan widget bisa kelihatan (sangat penting)
        self.opengl_widget.setMaximumSize(355, 350)

        # 3. Ambil layout yang berisi OpenGLWidget lama
        layout = self.ui.gridLayout_3

        # 2. Ganti widgetnya
        layout.replaceWidget(self.ui.openGLWidget, self.opengl_widget)

        # 3. Hapus placeholder lama
        self.ui.openGLWidget.setParent(None)

        # 4. Tampilkan ulang
        self.opengl_widget.show()

        #yaw_dot
        self.dot_pixmap = QPixmap("/home/techsas/auv_ws/src/auv_pkg/auv_pkg/gui/dot.png").scaled(
            390, 390,
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.FastTransformation
        )

        self.ui.labeldot.setScaledContents(False)

        self.ui.labeldot.setPixmap(self.dot_pixmap)

        # Inisialisasi yaw
        self.current_yaw = 0
        self.current_pitch = 0
        self.current_roll = 0

        self.rotate_timer = QtCore.QTimer()
        self.rotate_timer.timeout.connect(self.updateYaw)
        self.rotate_timer.start(50)
        
        # camera
        # self.camera_thread = CameraThread()
        # self.camera_thread.frame_ready.connect(self.update_camera_view)
        # self.camera_thread.start()


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

        self.ui.setPointStatusButton.clicked.connect(self.publish_values)
        self.ui.emergencyButton.clicked.connect(self.emergency_stop)

        self.get_logger().info("GUI ROS2 Node Started with Publishers")
    
    def emergency_stop(self):
        self.publish_values(status_override="stop")
    
    def publish_values(self, status_override=None):
        try:
            if isinstance(status_override, str):
                statusText = status_override
            else:
                statusText = self.ui.comboBoxStatus.currentText()
            
            yaw = float(self.ui.setYaw.text()) 
            depth = float(self.ui.setDepth.text())
            pitch = float(self.ui.setPitch.text())
            roll = float(self.ui.setRoll.text())

            kp_yaw = float(self.ui.setPYaw.text())
            kp_pitch = float(self.ui.setPPitch.text())
            kp_roll = float(self.ui.setPRoll.text())
            kp_depth = float(self.ui.setPDepth.text())
            kp_camera = float(self.ui.setPCamera.text())

            ki_yaw = float(self.ui.setIYaw.text())
            ki_pitch = float(self.ui.setIPitch.text())
            ki_roll = float(self.ui.setIRoll.text())
            ki_depth = float(self.ui.setIDepth.text())
            ki_camera = float(self.ui.setICamera.text())
            
            kd_yaw = float(self.ui.setDYaw.text())
            kd_pitch = float(self.ui.setDPitch.text())
            kd_roll = float(self.ui.setDRoll.text())
            kd_depth = float(self.ui.setDDepth.text())
            kd_camera = float(self.ui.setDCamera.text())
        except:
            print("ERROR: Input tidak valid")
            return

        # -------- PID values (bisa kamu ubah) ----------
        pid_yaw = PID()
        pid_yaw.kp = kp_yaw
        pid_yaw.ki = ki_yaw
        pid_yaw.kd = kd_yaw

        pid_pitch = PID()
        pid_pitch.kp = kp_pitch
        pid_pitch.ki = ki_pitch
        pid_pitch.kd = kd_pitch

        pid_roll = PID()
        pid_roll.kp = kp_roll
        pid_roll.ki = ki_roll
        pid_roll.kd = kd_roll

        pid_depth = PID()
        pid_depth.kp = kp_depth
        pid_depth.ki = ki_depth
        pid_depth.kd = kd_depth

        pid_camera = PID()
        pid_camera.kp = kp_camera
        pid_camera.ki = ki_camera
        pid_camera.kd = kd_camera

        multi_pid_msg = MultiPID()
        multi_pid_msg.pid_yaw = pid_yaw
        multi_pid_msg.pid_pitch = pid_pitch
        multi_pid_msg.pid_roll = pid_roll
        multi_pid_msg.pid_depth = pid_depth
        multi_pid_msg.pid_camera = pid_camera

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
    
    def g_to_deg(self, g_value, gain=2.5):
        g = max(min(g_value * gain, 1.0), -1.0)
        return math.degrees(math.asin(g))

    def updateYaw(self):
        angle = self.current_yaw

        pitch_deg = self.current_pitch*22.5
        roll_deg  = self.g_to_deg(self.current_roll)

        transform = QTransform().rotate(angle)
        rotated = self.dot_pixmap.transformed(transform, QtCore.Qt.FastTransformation)

        self.ui.labeldot.setPixmap(rotated)

        if self.opengl_widget:
            self.opengl_widget.update_orientation(-angle, -pitch_deg, roll_deg)


    def status_callback(self, msg):
        self.ui.Status.setText(msg.data)
    
    def status_setpoint_callback(self, msg):
        self.ui.statusSetPoint.setText(msg.data)

    def sensor_callback(self, msg):
        self.current_yaw = round(msg.yaw, 0)
        self.current_pitch = round(msg.pitch, 0)
        self.current_roll = round(msg.roll, 0)
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