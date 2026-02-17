#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import custom interfaces
from auv_interfaces.msg import PID, MultiPID, SetPoint, Actuator, Sensor
from std_msgs.msg import Float32

import serial
import threading
import json
import time


class PubTeensy(Node):

    def __init__(self):
        super().__init__("pub_teensy")
        self.get_logger().info("Starting Publisher Teensy Node (HWT905)")

        # ================= CONFIG SERIAL =================
        self.serial_port = '/dev/ttyACM0'  # Sesuaikan port
        self.baud_rate = 115200

        # ================= PUBLISHERS =================
        self.pub_yaw = self.create_publisher(Sensor, 'teensy/sensor', 10)
        self.pub_velocity = self.create_publisher(Float32, 'teensy/velocity', 10)
        self.pub_setpoint = self.create_publisher(SetPoint, 'teensy/setpoint', 10)
        self.pub_error = self.create_publisher(Float32, 'teensy/error', 10)
        self.pub_pid_total = self.create_publisher(Float32, 'teensy/pid_total', 10)
        self.pub_pwm = self.create_publisher(Float32, 'teensy/pwm', 10)

        # MultiPID & Actuator Publishers
        self.pub_multipid_velocity = self.create_publisher(MultiPID, 'teensy/multipid', 10)
        self.pub_thrusters = self.create_publisher(Actuator, 'teensy/actuators', 10)

        # ================= SERIAL CONNECT =================
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Serial connected to {self.serial_port}")
            time.sleep(2) # Tunggu Teensy restart
            
            # Start Thread Reader
            self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.serial_thread.start()
        except Exception as e:
            self.get_logger().error(f"Serial failed: {e}")
            self.ser = None

        # ================ INITIAL PARAMETERS (HARDCORE) =================
        # Setting awal agar langsung jalan saat node nyala
        setpoint = SetPoint()
        setpoint.yaw = -60.0  # Contoh target 10 derajat
        
        pid_yaw = PID()
        pid_yaw.kp = 3.0
        pid_yaw.ki = 0.0
        pid_yaw.kd = 0.125

        # Kirim parameter ke Teensy
        if self.ser is not None:
            self.get_logger().info("Sending Initial Parameters...")
            self.set_kp(pid_yaw.kp)
            time.sleep(0.05)
            self.set_ki(pid_yaw.ki)
            time.sleep(0.05)
            self.set_kd(pid_yaw.kd)
            time.sleep(0.05)
            self.set_setpoint(setpoint.yaw)

            self.get_logger().info(f"Sent SP:{setpoint.yaw} Kp:{pid_yaw.kp} Ki:{pid_yaw.ki} Kd:{pid_yaw.kd}")

    # ==========================================================
    # SERIAL READER
    # ==========================================================
    def serial_reader(self):
        if self.ser is None:
            return

        while rclpy.ok():
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                    # Filter JSON valid
                    if line.startswith('{') and line.endswith('}'):
                        try:
                            data = json.loads(line)
                        except json.JSONDecodeError:
                            continue

                        # -------- BASIC FLOATS --------
                        if 'yaw' in data:
                            sensor = Sensor()
                            sensor.yaw = float(data['yaw'])
                            self.pub_yaw.publish(sensor)

                        # DEBUG VELOCITY: Pastikan data['velocity'] ada
                        if 'velocity' in data:
                            msg = Float32()
                            msg.data = float(data['velocity'])
                            self.pub_velocity.publish(msg)

                        if 'setpoint' in data:
                            sp_msg = SetPoint()
                            sp_msg.yaw = float(data['setpoint'])
                            self.pub_setpoint.publish(sp_msg)

                        if 'error' in data:
                            msg = Float32()
                            msg.data = float(data['error'])
                            self.pub_error.publish(msg)

                        if 'total' in data:
                            msg = Float32()
                            msg.data = float(data['total'])
                            self.pub_pid_total.publish(msg)

                        if 'pwm' in data:
                            msg = Float32()
                            msg.data = float(data['pwm'])
                            self.pub_pwm.publish(msg)

                        # -------- MULTIPID (FEEDBACK) --------
                        if 'p' in data or 'i' in data:
                            pid_yaw = PID()
                            # Menampilkan aksi PID real-time
                            pid_yaw.kp = float(data.get('p', 0.0))
                            pid_yaw.ki = float(data.get('i', 0.0))
                            pid_yaw.kd = float(data.get('d', 0.0))

                            multi_msg = MultiPID()
                            multi_msg.pid_yaw = pid_yaw
                            # PID lain dummy dulu
                            multi_msg.pid_pitch = PID()
                            multi_msg.pid_roll = PID()
                            multi_msg.pid_depth = PID()
                            multi_msg.pid_camera = PID()

                            self.pub_multipid_velocity.publish(multi_msg)

                        # -------- THRUSTERS --------
                        if 'thrusters' in data:
                            thrusters = data['thrusters']
                            msg = Actuator()
                            
                            # Safely map array
                            if len(thrusters) >= 8:
                                msg.thruster_1 = float(thrusters[0])
                                msg.thruster_2 = float(thrusters[1])
                                msg.thruster_3 = float(thrusters[2])
                                msg.thruster_4 = float(thrusters[3])
                                msg.thruster_5 = float(thrusters[4])
                                msg.thruster_6 = float(thrusters[5])
                                msg.thruster_7 = float(thrusters[6])
                                msg.thruster_8 = float(thrusters[7])
                            
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
        if self.ser is None: return
        try:
            self.ser.write((cmd + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Send error: {e}")

    def set_setpoint(self, value): self.send_command(f"sp {value}")
    def set_kp(self, value): self.send_command(f"kp {value}")
    def set_ki(self, value): self.send_command(f"ki {value}")
    def set_kd(self, value): self.send_command(f"kd {value}")
    def stop_teensy(self): self.send_command("stop")


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