#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from auv_interfaces.msg import PID, MultiPID, SetPoint, Actuator, Sensor
from std_msgs.msg import String, Float32

import serial
import threading
import json
import time


class PubTeensy(Node):

    def __init__(self):
        super().__init__("pub_teensy")
        self.get_logger().info("Starting Publisher Teensy Node")

        # ================= PUBLISHERS =================
        # self.pub_status = self.create_publisher(String, 'teensy/status', 10)
        self.pub_yaw = self.create_publisher(Sensor, 'teensy/sensor', 10)
        self.pub_velocity = self.create_publisher(Float32, 'teensy/velocity', 10)
        self.pub_setpoint = self.create_publisher(SetPoint, 'teensy/setpoint', 10)
        self.pub_error = self.create_publisher(Float32, 'teensy/error', 10)
        self.pub_pid_total = self.create_publisher(Float32, 'teensy/pid_total', 10)
        self.pub_pwm = self.create_publisher(Float32, 'teensy/pwm', 10)

        # ✅ Publish MultiPID (BUKAN PID tunggal)
        self.pub_multipid_velocity = self.create_publisher(MultiPID, 'teensy/multipid', 10)

        self.pub_thrusters = self.create_publisher(Actuator, 'teensy/actuators', 10)

        # ================= SERIAL =================
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial connected")
            self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.serial_thread.start()
        except Exception as e:
            self.get_logger().error(f"Serial failed: {e}")
            self.ser = None

        # ================ INITIAL PARAMETERS SETPOINT =================
        setpoint = SetPoint()
        setpoint.yaw = 260.0
        setpoint.pitch = 0.0
        setpoint.roll = 0.0
        setpoint.depth = 0.0

        # ================ INITIAL PARAMETERS PID =================
        pid_yaw = PID()
        pid_yaw.kp = 3.0
        pid_yaw.ki = 0.0
        pid_yaw.kd = 0.5

        pid_pitch = PID()
        pid_pitch.kp = 0.0
        pid_pitch.ki = 0.0
        pid_pitch.kd = 0.0

        pid_roll = PID()    
        pid_roll.kp = 0.0
        pid_roll.ki = 0.0
        pid_roll.kd = 0.0

        pid_depth = PID()
        pid_depth.kp = 0.0
        pid_depth.ki = 0.0
        pid_depth.kd = 0.0

        pid_camera = PID()
        pid_camera.kp = 0.0
        pid_camera.ki = 0.0
        pid_camera.kd = 0.0

        # Create MultiPID message and add multiple PID sets
        multi_pid_msg = MultiPID()
        multi_pid_msg.pid_yaw = pid_yaw
        multi_pid_msg.pid_pitch = pid_pitch
        multi_pid_msg.pid_roll = pid_roll
        multi_pid_msg.pid_depth = pid_depth
        multi_pid_msg.pid_camera = pid_camera

        # Create Status message
        # status = String()
        # status.data = "yaw"

        # ================= SEND INITIAL DATA YAW =================
        self.set_setpoint(setpoint.yaw)

        # ================ SEND INITIAL DATA PID YAW ================
        self.set_kp(pid_yaw.kp)
        self.set_ki(pid_yaw.ki)
        self.set_kd(pid_yaw.kd)
        # self.set_status(status.data)

        self.get_logger().info("Initial parameters set to Teensy")
        self.get_logger().info("-------------INITIAL PARAMETERS-------------")
        # self.get_logger().info(f"Status: {status.data}")
        self.get_logger().info(f"SetPoint yaw: {setpoint.yaw}")
        self.get_logger().info(f"Kp: {pid_yaw.kp}")
        self.get_logger().info(f"Ki: {pid_yaw.ki}")
        self.get_logger().info(f"Kd: {pid_yaw.kd}")

        self.get_logger().info("Initial parameters sent")

    # ==========================================================
    # SERIAL READER
    # ==========================================================

    def serial_reader(self):

        if self.ser is None:
            return

        while rclpy.ok():
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode().strip()

                    if line.startswith('{') and line.endswith('}'):

                        data = json.loads(line)

                        # if 'status' in data:
                        #     status = String()
                        #     status.data = data['status']
                        #     self.pub_status.publish(status)

                        if 'yaw' in data:
                            sensor = Sensor()
                            sensor.yaw = float(data['yaw'])
                            self.pub_yaw.publish(sensor)

                        if 'velocity' in data:
                            msg = Float32()
                            msg.data = float(data['velocity'])
                            self.pub_velocity.publish(msg)

                        if 'setpoint' in data:
                            setpoint = SetPoint()
                            setpoint.yaw = float(data['setpoint'])
                            self.pub_setpoint.publish(setpoint)

                        if 'error' in data:
                            msg = Float32()
                            msg.data = float(data['error'])
                            self.pub_error.publish(msg)

                        if 'pwm' in data:
                            msg = Float32()
                            msg.data = float(data['pwm'])
                            self.pub_pwm.publish(msg)

                        # -------- MULTIPID --------
                        if 'p' in data or 'i' in data or 'd' in data:

                            pid_yaw = PID()
                            pid_yaw.kp = float(data.get('p', 0.0))
                            pid_yaw.ki = float(data.get('i', 0.0))
                            pid_yaw.kd = float(data.get('d', 0.0))

                            multi_msg = MultiPID()

                            multi_msg.pid_yaw = pid_yaw
                            multi_msg.pid_pitch = PID()
                            multi_msg.pid_roll = PID()
                            multi_msg.pid_depth = PID()
                            multi_msg.pid_camera = PID()

                            self.pub_multipid_velocity.publish(multi_msg)
                        
                        if 'total' in data:
                            msg = Float32()
                            msg.data = float(data['total'])
                            self.pub_pid_total.publish(msg)

                        # -------- THRUSTERS --------
                        if 'thrusters' in data:
                            thrusters = data['thrusters']

                            msg = Actuator()

                            if len(thrusters) >= 1: msg.thruster_1 = float(thrusters[0])
                            if len(thrusters) >= 2: msg.thruster_2 = float(thrusters[1])
                            if len(thrusters) >= 3: msg.thruster_3 = float(thrusters[2])
                            if len(thrusters) >= 4: msg.thruster_4 = float(thrusters[3])
                            if len(thrusters) >= 5: msg.thruster_5 = float(thrusters[4])
                            if len(thrusters) >= 6: msg.thruster_6 = float(thrusters[5])
                            if len(thrusters) >= 7: msg.thruster_7 = float(thrusters[6])
                            if len(thrusters) >= 8: msg.thruster_8 = float(thrusters[7])

                            msg.thruster_9 = 0.0
                            msg.thruster_10 = 0.0

                            self.pub_thrusters.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Serial reader error: {e}")
                time.sleep(0.1)

    # ==========================================================
    # SERIAL COMMANDS
    # ==========================================================

    def send_command(self, cmd):
        if self.ser is None:
            return

        try:
            self.ser.write((cmd + '\n').encode())
        except Exception as e:
            self.get_logger().error(f"Send error: {e}")

    def set_status(self, value):
        self.send_command(f"status {value}")

    def set_setpoint(self, value):
        self.send_command(f"sp {value}")

    def set_kp(self, value):
        self.send_command(f"kp {value}")

    def set_ki(self, value):
        self.send_command(f"ki {value}")

    def set_kd(self, value):
        self.send_command(f"kd {value}")

    def stop_teensy(self):
        self.send_command("stop")


def main(args=None):
    rclpy.init(args=args)
    node = PubTeensy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.stop_teensy()
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()